#pragma once

// Small 2.5D geometry toolbox used by the map renderer: polylines measured in
// the XY plane (z is carried along and interpolated), and a triangle mesh that
// turns into a single TRIANGLE_LIST marker.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "earcut.hpp"

namespace laneviz {

  struct Vec3 {
    double x{ 0.0 }, y{ 0.0 }, z{ 0.0 };
  };

  using Polyline = std::vector<Vec3>;

  struct Color {
    float r{ 1.0f }, g{ 1.0f }, b{ 1.0f };

    static constexpr Color hex(std::uint32_t rgb) {
      return Color{ ((rgb >> 16) & 0xFF) / 255.0f, ((rgb >> 8) & 0xFF) / 255.0f, (rgb & 0xFF) / 255.0f };
    }
    /// Linear blend: t = 0 gives a, t = 1 gives b.
    static Color mix(const Color& a, const Color& b, float t) {
      return Color{ a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t };
    }
    bool operator==(const Color& o) const { return r == o.r && g == o.g && b == o.b; }
  };

  inline double dist2d(const Vec3& a, const Vec3& b) { return std::hypot(b.x - a.x, b.y - a.y); }

  inline Vec3 lerp(const Vec3& a, const Vec3& b, double t) {
    return Vec3{ a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t };
  }

  /// Cumulative XY arc length of every vertex (same size as the polyline).
  inline std::vector<double> arcLengths(const Polyline& pl) {
    std::vector<double> s(pl.size(), 0.0);
    for (size_t i = 1; i < pl.size(); ++i) s[i] = s[i - 1] + dist2d(pl[i - 1], pl[i]);
    return s;
  }

  /// A polyline together with its arc length table, for repeated sampling.
  struct MeasuredLine {
    Polyline pts;
    std::vector<double> s;

    MeasuredLine() = default;
    explicit MeasuredLine(Polyline p) : pts(std::move(p)), s(arcLengths(pts)) {}

    double length() const { return s.empty() ? 0.0 : s.back(); }

    Vec3 at(double d) const {
      if (pts.empty()) return {};
      if (d <= 0.0) return pts.front();
      if (d >= length()) return pts.back();
      const size_t i = std::upper_bound(s.begin(), s.end(), d) - s.begin();  // s[i-1] <= d < s[i]
      const double seg = s[i] - s[i - 1];
      return lerp(pts[i - 1], pts[i], seg > 1e-9 ? (d - s[i - 1]) / seg : 0.0);
    }

    /// Part of the line between two arc lengths, keeping the original vertices in between.
    Polyline sub(double d0, double d1) const {
      Polyline out;
      d0 = std::max(d0, 0.0);
      d1 = std::min(d1, length());
      if (pts.size() < 2 || d1 - d0 < 1e-6) return out;
      out.push_back(at(d0));
      for (size_t i = 0; i < pts.size(); ++i) {
        if (s[i] > d0 + 1e-6 && s[i] < d1 - 1e-6) out.push_back(pts[i]);
      }
      out.push_back(at(d1));
      return out;
    }

    /// Unit XY direction of travel around arc length d.
    Vec3 direction(double d, double span = 0.5) const {
      const Vec3 a = at(d - span), b = at(d + span);
      const double l = dist2d(a, b);
      return l > 1e-9 ? Vec3{ (b.x - a.x) / l, (b.y - a.y) / l, 0.0 } : Vec3{ 1.0, 0.0, 0.0 };
    }
  };

  inline bool pointInPolygon(const Vec3& p, const Polyline& poly) {
    bool inside = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
      const Vec3& a = poly[i];
      const Vec3& b = poly[j];
      if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
    }
    return inside;
  }

  inline double distToSegment(const Vec3& p, const Vec3& a, const Vec3& b) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double l2 = dx * dx + dy * dy;
    const double t = l2 > 1e-12 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / l2, 0.0, 1.0) : 0.0;
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
  }

  /// Distance from p to the closed outline of a polygon.
  inline double distToOutline(const Vec3& p, const Polyline& poly) {
    double best = 1e18;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) best = std::min(best, distToSegment(p, poly[i], poly[j]));
    return best;
  }

  /// 2D rigid frame: local +x is "forward", local +y is "left".
  struct Frame {
    Vec3 origin;
    Vec3 fwd{ 1.0, 0.0, 0.0 };
    double scale{ 1.0 };

    Vec3 operator()(double lx, double ly) const {
      lx *= scale;
      ly *= scale;
      return Vec3{ origin.x + fwd.x * lx - fwd.y * ly, origin.y + fwd.y * lx + fwd.x * ly, origin.z };
    }
  };

  /// Triangle soup with per-vertex colour; becomes one TRIANGLE_LIST marker.
  class Mesh {
  public:
    bool empty() const { return pts_.empty(); }
    size_t vertexCount() const { return pts_.size(); }

    /// Triangles are always stored counter-clockwise seen from above. RViz
    /// lights a triangle by its winding, so a clockwise one facing the ground
    /// would be drawn black.
    void triangle(const Vec3& a, const Vec3& b, const Vec3& c, const Color& col, double dz = 0.0) {
      const bool ccw = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) >= 0.0;
      push(a, col, dz);
      push(ccw ? b : c, col, dz);
      push(ccw ? c : b, col, dz);
    }

    void quad(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, const Color& col, double dz = 0.0) {
      triangle(a, b, c, col, dz);
      triangle(a, c, d, col, dz);
    }

    /// Filled simple polygon (ear clipping in XY, z taken from the vertices).
    void polygon(const Polyline& ring, const Color& col, double dz = 0.0) {
      if (ring.size() < 3) return;
      std::vector<std::vector<std::array<double, 2>>> rings(1);
      rings[0].reserve(ring.size());
      for (const auto& p : ring) rings[0].push_back({ p.x, p.y });
      const std::vector<std::uint32_t> idx = mapbox::earcut<std::uint32_t>(rings);
      for (size_t i = 0; i + 2 < idx.size(); i += 3) triangle(ring[idx[i]], ring[idx[i + 1]], ring[idx[i + 2]], col, dz);
    }

    /// Constant-width stroke along a polyline with mitred joins.
    void ribbon(const Polyline& pl, double width, const Color& col, double dz = 0.0, bool closed = false) {
      const size_t n = pl.size();
      if (n < 2) return;
      const double h = width / 2.0;
      std::vector<Vec3> lft(n), rgt(n);
      for (size_t i = 0; i < n; ++i) {
        const bool has_prev = closed || i > 0, has_next = closed || i + 1 < n;
        const Vec3& prev = pl[has_prev ? (i + n - 1) % n : i];
        const Vec3& next = pl[has_next ? (i + 1) % n : i];
        double ax = pl[i].x - prev.x, ay = pl[i].y - prev.y, bx = next.x - pl[i].x, by = next.y - pl[i].y;
        const double al = std::hypot(ax, ay), bl = std::hypot(bx, by);
        if (al > 1e-9) { ax /= al; ay /= al; } else { ax = bx / std::max(bl, 1e-9); ay = by / std::max(bl, 1e-9); }
        if (bl > 1e-9) { bx /= bl; by /= bl; } else { bx = ax; by = ay; }
        // Miter: normal of the averaged direction, lengthened to keep the stroke width (capped).
        double tx = ax + bx, ty = ay + by;
        const double tl = std::hypot(tx, ty);
        if (tl < 1e-6) { tx = ax; ty = ay; } else { tx /= tl; ty /= tl; }
        const double cos_half = std::max(tx * ax + ty * ay, 0.5);
        const double nx = -ty * h / cos_half, ny = tx * h / cos_half;
        lft[i] = Vec3{ pl[i].x + nx, pl[i].y + ny, pl[i].z };
        rgt[i] = Vec3{ pl[i].x - nx, pl[i].y - ny, pl[i].z };
      }
      const size_t segs = closed ? n : n - 1;
      for (size_t i = 0; i < segs; ++i) {
        const size_t j = (i + 1) % n;
        quad(lft[i], rgt[i], rgt[j], lft[j], col, dz);
      }
    }

    /// Regular n-gon lying flat, e.g. a sign symbol.
    void disc(const Vec3& c, double radius, int sides, const Color& col, double dz = 0.0, double phase = 0.0) {
      for (int i = 0; i < sides; ++i) {
        const double a0 = phase + 2.0 * M_PI * i / sides, a1 = phase + 2.0 * M_PI * (i + 1) / sides;
        triangle(c,
          Vec3{ c.x + radius * std::cos(a0), c.y + radius * std::sin(a0), c.z },
          Vec3{ c.x + radius * std::cos(a1), c.y + radius * std::sin(a1), c.z }, col, dz);
      }
    }

    visualization_msgs::msg::Marker toMarker(const std::string& frame_id, const std::string& ns) const {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id;  // stamp stays zero: the map is static, use the latest transform
      m.ns = ns;
      m.id = 0;
      m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 1.0;
      m.color.r = m.color.g = m.color.b = m.color.a = 1.0f;
      m.points = pts_;
      if (uniform_) {
        if (!cols_.empty()) m.color = cols_.front();  // one colour: skip the per-vertex array
      }
      else {
        m.colors = cols_;
      }
      return m;
    }

  private:
    void push(const Vec3& p, const Color& c, double dz) {
      geometry_msgs::msg::Point gp;
      gp.x = p.x;
      gp.y = p.y;
      gp.z = p.z + dz;
      pts_.push_back(gp);
      std_msgs::msg::ColorRGBA gc;
      gc.r = c.r;
      gc.g = c.g;
      gc.b = c.b;
      gc.a = 1.0f;
      if (!cols_.empty() && uniform_) {
        const auto& f = cols_.front();
        uniform_ = f.r == gc.r && f.g == gc.g && f.b == gc.b;
      }
      cols_.push_back(gc);
    }

    std::vector<geometry_msgs::msg::Point> pts_;
    std::vector<std_msgs::msg::ColorRGBA> cols_;
    bool uniform_{ true };
  };

}  // namespace laneviz
