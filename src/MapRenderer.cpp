#include "laneletvisualizer/MapRenderer.hpp"

#include <map>
#include <tuple>
#include <unordered_map>

namespace laneviz {

  namespace {

    // Layer heights above the road surface [m]. Big enough to avoid z-fighting
    // in a perspective view, small enough to be invisible.
    constexpr double Z_PARKING = -0.02;
    constexpr double Z_ROAD = 0.0;
    constexpr double Z_SPEED = 0.02;
    constexpr double Z_SURFACE_MARK = 0.04;  // chevrons, junction paths
    constexpr double Z_LINE = 0.06;
    constexpr double Z_PAINT = 0.08;  // stop lines, zebra, arrows
    constexpr double Z_SIGN = 0.12;

    constexpr double SAMPLE_STEP = 0.25;    // boundary visibility sampling [m]
    constexpr double COVER_MARGIN = 0.4;    // "well inside another lane" distance [m]
    constexpr double EDGE_PROBE = 0.6;      // how far beside a line to look for road [m]
    constexpr double MIN_RUN = 1.0;         // shortest visible piece of a partly hidden line [m]
    constexpr double ARROW_LENGTH = 3.2;    // pavement arrow glyph length [m]
    constexpr double ARROW_SETBACK = 2.5;   // arrow tip to lane end [m]
    constexpr double ARROW_MIN_LANE = 10.0; // shortest lane that gets an arrow [m]

    template <typename Primitive>
    std::string attr(const Primitive& p, const char* key) {
      return p.hasAttribute(key) ? p.attribute(key).value() : std::string();
    }

    template <typename Primitive>
    bool isYes(const Primitive& p, const char* key) {
      const std::string v = attr(p, key);
      return v == "yes" || v == "true" || v == "1";
    }

    Vec3 midpoint(const Vec3& a, const Vec3& b) { return lerp(a, b, 0.5); }

    /// Twice the signed XY area of the ring "left bound, then right bound backwards".
    double ringArea(const Polyline& left, const Polyline& right, bool flip_left, bool flip_right) {
      Polyline ring(left);
      if (flip_left) std::reverse(ring.begin(), ring.end());
      if (flip_right) ring.insert(ring.end(), right.begin(), right.end());
      else ring.insert(ring.end(), right.rbegin(), right.rend());
      double area = 0.0;
      for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) area += ring[j].x * ring[i].y - ring[i].x * ring[j].y;
      return area;
    }

    /// Which bounds have to be reversed so that both run in the driving
    /// direction with the left bound on the left.
    ///
    /// Lanelet2 aligns the bounds while loading, but it does so with the
    /// projected lat / lon coordinates. Our maps only carry usable coordinates
    /// in local_x / local_y, so that alignment is arbitrary and is redone here.
    std::pair<bool, bool> boundFlips(const Polyline& left, const Polyline& right) {
      const double as_is = dist2d(left.front(), right.front()) + dist2d(left.back(), right.back());
      const double crossed = dist2d(left.front(), right.back()) + dist2d(left.back(), right.front());
      const bool aligned = as_is <= crossed;
      // Two candidates have the bounds parallel; in the right one the ring
      // "left, then right backwards" runs clockwise.
      const bool flip_left = ringArea(left, right, false, !aligned) > 0.0;
      return { flip_left, flip_left == aligned };
    }

    /// Pavement arrow in a local frame: origin at the tail, +x forward, ARROW_LENGTH long.
    void arrowGlyph(Mesh& mesh, const Frame& f, bool left, bool straight, bool right, const Color& col, double dz) {
      const double w = 0.2 * f.scale;
      const auto head = [&](double bx, double by, double dx, double dy, double len, double half) {
        const double l = std::hypot(dx, dy);
        dx /= l;
        dy /= l;
        mesh.triangle(f(bx - dy * half, by + dx * half), f(bx + dy * half, by - dx * half), f(bx + dx * len, by + dy * len), col, dz);
      };
      if (straight) {
        mesh.ribbon({ f(0.0, 0.0), f(2.15, 0.0) }, w, col, dz);
        head(2.1, 0.0, 1.0, 0.0, ARROW_LENGTH - 2.1, 0.38);
      }
      const double x0 = straight ? 0.8 : 1.6;  // where the side branch leaves the shaft
      if (!straight && (left || right)) mesh.ribbon({ f(0.0, 0.0), f(x0, 0.0) }, w, col, dz);
      for (const double sg : { 1.0, -1.0 }) {
        if (!(sg > 0 ? left : right)) continue;
        mesh.ribbon({ f(x0 - 0.1, 0.0), f(x0 + 0.25, 0.08 * sg), f(x0 + 0.5, 0.3 * sg), f(x0 + 0.62, 0.62 * sg) }, w, col, dz);
        head(x0 + 0.6, 0.58 * sg, 0.3, 1.0 * sg, 0.85, 0.38);
      }
    }

  }  // namespace

  // ---------------------------------------------------------------------------
  // Construction: analyse the map once, build every layer once.
  // ---------------------------------------------------------------------------

  MapRenderer::MapRenderer(const lanelet::LaneletMap& map, const Style& style) : style_(style) {
    collectLanes(map);
    connectLanes();
    detectJunctions();
    collectBoundaries(map);

    drawParking(map);
    drawSurfaces();
    drawTurnArrows();  // before the chevrons, which keep clear of the arrows
    drawChevrons();
    drawJunctionPaths();
    drawBoundaries();
    drawCrosswalks(map);
    drawStopLinesAndSigns(map);
  }

  visualization_msgs::msg::MarkerArray MapRenderer::markers(const std::string& frame_id) const {
    visualization_msgs::msg::MarkerArray out;
    for (const Layer* layer : layers()) {
      auto m = layer->mesh.toMarker(frame_id, layer->ns);
      // Keep the namespace alive even when a map has nothing for it, so a
      // reloaded map clears what the previous one drew there.
      if (layer->mesh.empty()) m.action = visualization_msgs::msg::Marker::DELETE;
      out.markers.push_back(std::move(m));
    }
    return out;
  }

  size_t MapRenderer::vertexCount() const {
    size_t n = 0;
    for (const Layer* layer : layers()) n += layer->mesh.vertexCount();
    return n;
  }

  Polyline MapRenderer::toPolyline(const lanelet::ConstLineString3d& ls) const {
    Polyline out;
    out.reserve(ls.size());
    for (const auto& p : ls) out.push_back(Vec3{ p.x(), p.y(), style_.use_elevation ? p.z() : 0.0 });
    return out;
  }

  // ---------------------------------------------------------------------------
  // Map analysis
  // ---------------------------------------------------------------------------

  void MapRenderer::collectLanes(const lanelet::LaneletMap& map) {
    lanes_.reserve(map.laneletLayer.size());
    for (const auto& llt : map.laneletLayer) {
      if (attr(llt, "subtype") == "crosswalk" || attr(llt, "subtype") == "walkway") continue;
      const auto lb = llt.leftBound(), rb = llt.rightBound();
      if (lb.size() < 2 || rb.size() < 2) continue;

      Lane lane;
      lane.id = llt.id();
      Polyline left = toPolyline(lb), right = toPolyline(rb);
      lane.left_ls = lb.id();
      lane.right_ls = rb.id();
      lane.start_l = lb.front().id();
      lane.start_r = rb.front().id();
      lane.end_l = lb.back().id();
      lane.end_r = rb.back().id();
      const auto [flip_left, flip_right] = boundFlips(left, right);
      if (flip_left) {
        std::reverse(left.begin(), left.end());
        std::swap(lane.start_l, lane.end_l);
      }
      if (flip_right) {
        std::reverse(right.begin(), right.end());
        std::swap(lane.start_r, lane.end_r);
      }
      lane.left = MeasuredLine(std::move(left));
      lane.right = MeasuredLine(std::move(right));
      lane.turn = attr(llt, "turn_direction");
      if (llt.hasAttribute("speed_limit")) lane.speed = llt.attribute("speed_limit").asDouble().value_or(-1.0);

      lane.outline = lane.left.pts;
      lane.outline.insert(lane.outline.end(), lane.right.pts.rbegin(), lane.right.pts.rend());
      lane.min_x = lane.min_y = 1e18;
      lane.max_x = lane.max_y = -1e18;
      for (const auto& p : lane.outline) {
        lane.min_x = std::min(lane.min_x, p.x);
        lane.max_x = std::max(lane.max_x, p.x);
        lane.min_y = std::min(lane.min_y, p.y);
        lane.max_y = std::max(lane.max_y, p.y);
      }

      // Centerline: pair the bounds up by normalised arc length.
      const double ll = lane.left.length(), rl = lane.right.length();
      const int n = std::max(2, static_cast<int>(std::ceil(std::max(ll, rl) / 0.5)) + 1);
      Polyline c;
      c.reserve(n);
      for (int k = 0; k < n; ++k) {
        const double t = static_cast<double>(k) / (n - 1);
        c.push_back(midpoint(lane.left.at(t * ll), lane.right.at(t * rl)));
      }
      lane.center = MeasuredLine(std::move(c));
      lanes_.push_back(std::move(lane));
    }
  }

  void MapRenderer::connectLanes() {
    // B follows A when B starts on the two points A ends on.
    std::map<std::pair<lanelet::Id, lanelet::Id>, std::vector<int>> by_start;
    for (size_t i = 0; i < lanes_.size(); ++i) by_start[{ lanes_[i].start_l, lanes_[i].start_r }].push_back(static_cast<int>(i));
    for (auto& lane : lanes_) {
      const auto it = by_start.find({ lane.end_l, lane.end_r });
      if (it != by_start.end()) lane.successors = it->second;
    }
  }

  bool MapRenderer::coveredByOtherLane(const Vec3& p, const std::vector<int>& own, double margin) const {
    for (size_t i = 0; i < lanes_.size(); ++i) {
      const Lane& o = lanes_[i];
      if (p.x < o.min_x || p.x > o.max_x || p.y < o.min_y || p.y > o.max_y) continue;
      if (std::find(own.begin(), own.end(), static_cast<int>(i)) != own.end()) continue;
      if (pointInPolygon(p, o.outline) && distToOutline(p, o.outline) > margin) return true;
    }
    return false;
  }

  bool MapRenderer::onRoad(const Vec3& p) const {
    for (const Lane& o : lanes_) {
      if (p.x < o.min_x || p.x > o.max_x || p.y < o.min_y || p.y > o.max_y) continue;
      if (pointInPolygon(p, o.outline)) return true;
    }
    return false;
  }

  void MapRenderer::detectJunctions() {
    // A lane is part of a junction if the map says so (turn_direction), or if
    // most of its centerline runs across some other lane.
    for (size_t i = 0; i < lanes_.size(); ++i) {
      Lane& lane = lanes_[i];
      if (!lane.turn.empty()) {
        lane.junction = true;
        continue;
      }
      const double len = lane.center.length();
      int covered = 0, total = 0;
      for (double d = 0.1 * len; d <= 0.9 * len; d += 1.0, ++total) {
        if (coveredByOtherLane(lane.center.at(d), { static_cast<int>(i) }, COVER_MARGIN)) ++covered;
      }
      lane.junction = total > 0 && covered * 2 > total;
    }
  }

  void MapRenderer::collectBoundaries(const lanelet::LaneletMap& map) {
    std::unordered_map<lanelet::Id, int> by_id;
    // Separate linestrings drawn on top of each other (e.g. both directions
    // carrying their own copy of the centre line) are merged into one.
    std::map<std::tuple<long, long, long, long, long>, int> by_shape;
    const auto q = [](double v) { return std::lround(v / 0.2); };

    const auto add = [&](const lanelet::ConstLineString3d& ls, int lane_index) {
      if (attr(ls, "type") == "virtual") return;
      const auto known = by_id.find(ls.id());
      if (known != by_id.end()) {
        boundaries_[known->second].users.push_back(lane_index);
        return;
      }
      const bool crossable = attr(ls, "subtype") == "dashed" || isYes(ls, "lane_change");
      MeasuredLine line(toPolyline(ls));
      Vec3 a = line.pts.front(), b = line.pts.back();
      if (std::tie(b.x, b.y) < std::tie(a.x, a.y)) std::swap(a, b);
      const auto key = std::make_tuple(q(a.x), q(a.y), q(b.x), q(b.y), std::lround(line.length()));
      const auto twin = by_shape.find(key);
      if (twin != by_shape.end()) {
        Boundary& bd = boundaries_[twin->second];
        bd.users.push_back(lane_index);
        bd.crossable = bd.crossable && crossable;  // one solid line is enough to forbid crossing
        by_id[ls.id()] = twin->second;
        return;
      }
      Boundary bd;
      bd.line = std::move(line);
      bd.crossable = crossable;
      bd.color = attr(ls, "color") == "yellow" ? style_.marking_yellow : style_.marking;
      bd.users.push_back(lane_index);
      by_id[ls.id()] = by_shape[key] = static_cast<int>(boundaries_.size());
      boundaries_.push_back(std::move(bd));
    };

    std::unordered_map<lanelet::Id, int> lane_index;
    for (size_t i = 0; i < lanes_.size(); ++i) lane_index[lanes_[i].id] = static_cast<int>(i);
    for (const auto& llt : map.laneletLayer) {
      const auto it = lane_index.find(llt.id());
      if (it == lane_index.end()) continue;
      add(llt.leftBound(), it->second);
      add(llt.rightBound(), it->second);
    }
  }

  char MapRenderer::turnOf(const Lane& lane) const {
    if (lane.turn == "left") return 'l';
    if (lane.turn == "right") return 'r';
    if (lane.turn == "straight") return 's';
    const double len = lane.center.length();
    const Vec3 a = lane.center.direction(0.5), b = lane.center.direction(len - 0.5);
    const double angle = std::atan2(a.x * b.y - a.y * b.x, a.x * b.x + a.y * b.y);
    return angle > M_PI / 6 ? 'l' : angle < -M_PI / 6 ? 'r' : 's';
  }

  Color MapRenderer::speedColor(double speed) const {
    const double half = style_.speed_color_max / 2.0;
    if (speed <= 0.0) return style_.speed_slow;
    if (speed >= style_.speed_color_max) return style_.speed_fast;
    if (speed <= half) return Color::mix(style_.speed_slow, style_.speed_mid, static_cast<float>(speed / half));
    return Color::mix(style_.speed_mid, style_.speed_fast, static_cast<float>((speed - half) / half));
  }

  // ---------------------------------------------------------------------------
  // Layers
  // ---------------------------------------------------------------------------

  void MapRenderer::drawSurfaces() {
    for (const Lane& lane : lanes_) {
      road_.mesh.polygon(lane.outline, style_.asphalt, Z_ROAD);
      // Junction lanes overlap each other, often with different limits; leave them plain.
      if (lane.speed >= 0.0 && !lane.junction) speed_.mesh.polygon(lane.outline, Color::mix(style_.asphalt, speedColor(lane.speed), 0.22f), Z_SPEED);
    }
  }

  void MapRenderer::drawBoundaries() {
    for (const Boundary& bd : boundaries_) {
      const double len = bd.line.length();
      if (len < 1e-3) continue;

      // Visible stretches: where the line is not swallowed by another lane's
      // surface (that happens inside junctions and merges).
      const bool in_junction = std::all_of(bd.users.begin(), bd.users.end(), [this](int i) { return lanes_[i].junction; });
      std::vector<std::pair<double, double>> runs;
      const int n = std::max(1, static_cast<int>(std::ceil(len / SAMPLE_STEP)));
      double run_start = -1.0;
      for (int k = 0; k <= n; ++k) {
        const double d = len * k / n;
        const Vec3 p = bd.line.at(d);
        bool visible;
        if (in_junction) {
          // Inside a junction only the outer edge of the road is drawn.
          // "Outer" means one side is off the road, also a bit further out, so
          // that slivers between badly fitting lanelets do not count.
          const Vec3 t = bd.line.direction(d, SAMPLE_STEP);
          const auto off_road = [&](double side) {
            return !onRoad(Vec3{ p.x - t.y * side, p.y + t.x * side, p.z }) &&
              !onRoad(Vec3{ p.x - t.y * 2.0 * side, p.y + t.x * 2.0 * side, p.z });
          };
          visible = off_road(EDGE_PROBE) || off_road(-EDGE_PROBE);
        }
        else {
          visible = !coveredByOtherLane(p, bd.users, COVER_MARGIN);
        }
        if (visible && run_start < 0.0) run_start = d;
        if (run_start >= 0.0 && (!visible || k == n)) {
          const double run_end = visible ? d : len * (k - 1) / n;
          // Short leftovers are sampling noise around lane corners, not markings.
          if (run_end - run_start >= std::min(MIN_RUN, 0.999 * len)) runs.emplace_back(run_start, run_end);
          run_start = -1.0;
        }
      }

      if (!bd.crossable) {
        for (const auto& [a, b] : runs) lane_lines_.mesh.ribbon(bd.line.sub(a, b), style_.line_width, bd.color, Z_LINE);
        continue;
      }

      // Dashed: fit a whole number of periods into the line and centre the
      // pattern. That makes it independent of the drawing direction, keeps it
      // continuous across consecutive lanelets and lines up neighbouring lines.
      const double nominal = style_.dash_length + style_.dash_gap;
      const int periods = std::max(1, static_cast<int>(std::lround(len / nominal)));
      const double period = len / periods;
      const double dash = period * style_.dash_length / nominal;
      for (int k = 0; k < periods; ++k) {
        const double d0 = k * period + (period - dash) / 2.0, d1 = d0 + dash;
        for (const auto& [a, b] : runs) {
          const double s0 = std::max(d0, a), s1 = std::min(d1, b);
          if (s1 - s0 > 0.05) lane_lines_.mesh.ribbon(bd.line.sub(s0, s1), style_.line_width, bd.color, Z_LINE);
        }
      }
    }
  }

  void MapRenderer::drawJunctionPaths() {
    for (const Lane& lane : lanes_) {
      if (!lane.junction) continue;
      const double len = lane.center.length();
      if (len < 1.0) continue;
      junction_paths_.mesh.ribbon(lane.center.pts, 0.1, style_.junction_path, Z_SURFACE_MARK);
      const double at = 0.55 * len;
      const Frame f{ lane.center.at(at), lane.center.direction(at), 1.0 };
      junction_paths_.mesh.triangle(f(-0.35, 0.28), f(-0.35, -0.28), f(0.45, 0.0), style_.junction_path, Z_SURFACE_MARK);
    }
  }

  void MapRenderer::drawChevrons() {
    for (const Lane& lane : lanes_) {
      if (lane.junction) continue;
      const double len = lane.center.length();
      if (len < 10.0) continue;
      const int count = std::max(1, static_cast<int>(std::lround(len / 30.0)));
      for (int k = 0; k < count; ++k) {
        const double d = (k + 0.5) * len / count;
        if (lane.has_arrow && d > len - ARROW_SETBACK - ARROW_LENGTH - 3.0) continue;
        const double t = d / len;
        const double width = dist2d(lane.left.at(t * lane.left.length()), lane.right.at(t * lane.right.length()));
        const Frame f{ lane.center.at(d), lane.center.direction(d, 1.0), std::clamp(width / 3.2, 0.4, 1.2) };
        for (const double sg : { 1.0, -1.0 }) {
          direction_.mesh.quad(f(0.7, 0.0), f(-0.1, 0.6 * sg), f(-0.55, 0.6 * sg), f(0.25, 0.0), style_.chevron, Z_SURFACE_MARK);
        }
      }
    }
  }

  void MapRenderer::drawTurnArrows() {
    for (Lane& lane : lanes_) {
      if (lane.junction || lane.successors.empty()) continue;
      bool left = false, straight = false, right = false, tagged_turn = false;
      for (const int s : lane.successors) {
        const Lane& next = lanes_[s];
        tagged_turn = tagged_turn || next.turn == "left" || next.turn == "right";
        const char t = turnOf(next);
        left = left || t == 'l';
        right = right || t == 'r';
        straight = straight || t == 's';
      }
      // Only in front of mapped turns, and only on a straight run-up, the way
      // arrows are painted on real roads (so not inside a roundabout).
      if (!tagged_turn || !(left || right)) continue;
      const double len = lane.center.length();
      if (len < ARROW_MIN_LANE) continue;
      const double probe = std::min(len, 12.0);
      const Vec3 h0 = lane.center.direction(len - probe + 0.5), h1 = lane.center.direction(len - 0.5);
      if (h0.x * h1.x + h0.y * h1.y < std::cos(12.0 * M_PI / 180.0)) continue;
      const double tail = len - std::min(ARROW_SETBACK, (len - ARROW_LENGTH) / 2.0) - ARROW_LENGTH;
      const double t = (tail + ARROW_LENGTH / 2.0) / len;
      const double width = dist2d(lane.left.at(t * lane.left.length()), lane.right.at(t * lane.right.length()));
      const Vec3 a = lane.center.at(tail), b = lane.center.at(tail + ARROW_LENGTH);
      const double l = std::max(dist2d(a, b), 1e-6);
      const Frame f{ a, Vec3{ (b.x - a.x) / l, (b.y - a.y) / l, 0.0 }, std::clamp(0.8 * width / 2.9, 0.4, 1.0) };
      arrowGlyph(turn_arrows_.mesh, f, left, straight, right, style_.marking, Z_PAINT);
      lane.has_arrow = true;
    }
  }

  void MapRenderer::drawCrosswalks(const lanelet::LaneletMap& map) {
    constexpr double STRIPE = 0.5, PERIOD = 1.0;
    for (const auto& llt : map.laneletLayer) {
      if (attr(llt, "subtype") != "crosswalk") continue;
      Polyline lp = toPolyline(llt.leftBound()), rp = toPolyline(llt.rightBound());
      if (lp.size() < 2 || rp.size() < 2) continue;
      const auto [flip_left, flip_right] = boundFlips(lp, rp);
      if (flip_left != flip_right) std::reverse(rp.begin(), rp.end());  // only has to be parallel here
      const MeasuredLine left(std::move(lp)), right(std::move(rp));
      const double ll = left.length(), rl = right.length(), len = std::max(ll, rl);
      if (len < PERIOD) continue;

      // Stripes run across the walking direction. Mapped crosswalks often
      // stick out past the kerb, so only the stripes on a road are painted.
      struct Stripe { Vec3 a, b, c, d; bool on_road; };
      std::vector<Stripe> stripes;
      const int n = static_cast<int>(len / PERIOD);
      const double first = (len - n * PERIOD + (PERIOD - STRIPE)) / 2.0;
      bool any_on_road = false;
      for (int k = 0; k < n; ++k) {
        const double t0 = (first + k * PERIOD) / len, t1 = (first + k * PERIOD + STRIPE) / len;
        Stripe s{ left.at(t0 * ll), right.at(t0 * rl), right.at(t1 * rl), left.at(t1 * ll), false };
        s.on_road = onRoad(midpoint(midpoint(s.a, s.b), midpoint(s.c, s.d)));
        any_on_road = any_on_road || s.on_road;
        stripes.push_back(s);
      }
      for (const Stripe& s : stripes) {
        if (s.on_road || !any_on_road) crosswalk_.mesh.quad(s.a, s.b, s.c, s.d, style_.marking, Z_PAINT);
      }
    }
  }

  void MapRenderer::drawStopLinesAndSigns(const lanelet::LaneletMap& map) {
    for (const auto& ls : map.lineStringLayer) {
      const std::string type = attr(ls, "type");
      if (type == "stop_line" && ls.size() >= 2) {
        stop_lines_.mesh.ribbon(toPolyline(ls), 0.4, style_.marking, Z_PAINT);
      }
      else if (type == "traffic_sign" && !ls.empty()) {
        const Polyline pl = toPolyline(ls);
        const Vec3 c = midpoint(pl.front(), pl.back());
        const bool stop = attr(ls, "subtype") == "stop_sign";
        const int sides = stop ? 8 : 24;
        const double phase = stop ? M_PI / 8 : 0.0;
        traffic_signs_.mesh.disc(c, 0.62, sides, style_.marking, Z_SIGN, phase);
        traffic_signs_.mesh.disc(c, 0.5, sides, stop ? style_.sign_stop : style_.sign_other, Z_SIGN + 0.01, phase);
      }
    }
  }

  void MapRenderer::drawParking(const lanelet::LaneletMap& map) {
    const Color space_fill = Color::mix(style_.parking_fill, style_.parking_line, 0.18f);
    for (const auto& poly : map.polygonLayer) {
      if (attr(poly, "type") != "parking_lot" || poly.size() < 3) continue;
      Polyline ring;
      for (const auto& p : poly) ring.push_back(Vec3{ p.x(), p.y(), style_.use_elevation ? p.z() : 0.0 });
      parking_.mesh.polygon(ring, style_.parking_fill, Z_PARKING);
      parking_.mesh.ribbon(ring, 0.12, style_.parking_line, Z_PARKING + 0.002, /*closed=*/true);
    }
    // A parking space is mapped as its centre line plus a width.
    for (const auto& ls : map.lineStringLayer) {
      if (attr(ls, "type") != "parking_space" || ls.size() < 2) continue;
      const Polyline pl = toPolyline(ls);
      const Vec3 a = pl.front(), b = pl.back();
      const double len = dist2d(a, b);
      if (len < 1e-3) continue;
      const double half = (ls.hasAttribute("width") ? ls.attribute("width").asDouble().value_or(2.5) : 2.5) / 2.0;
      const Frame f{ a, Vec3{ (b.x - a.x) / len, (b.y - a.y) / len, 0.0 }, 1.0 };
      Polyline box{ f(0.0, half), f(len, half), f(len, -half), f(0.0, -half) };
      box[1].z = box[2].z = b.z;
      parking_.mesh.polygon(box, space_fill, Z_PARKING + 0.004);
      parking_.mesh.ribbon(box, 0.1, style_.parking_line, Z_PARKING + 0.008, /*closed=*/true);
    }
  }

}  // namespace laneviz
