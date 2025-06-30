#pragma once

#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include "triangulate_polygon.hpp"

namespace laneviz {

  class Lanelet {
  public:
    explicit Lanelet(const lanelet::ConstLanelet& llt) noexcept
      : _llt(&llt) {}

    lanelet::Id id() const noexcept { return _llt->id(); }

    lanelet::ConstLineString3d leftBound() const noexcept { return _llt->leftBound(); }
    lanelet::ConstLineString3d rightBound() const noexcept { return _llt->rightBound(); }

    const std::string& location() const { return _llt->attributes().at("location").value(); }
    bool oneWay() const { return _llt->attributes().at("one_way").asBool().value(); }
    double speedLimit() const { return _llt->attributes().at("speed_limit").asDouble().value(); }
    const std::string& subtype() const { return _llt->attributes().at("subtype").value(); }
    const std::string& turnDirection() const { return _llt->attributes().at("turn_direction").value(); }

    visualization_msgs::msg::MarkerArray draw(
      const std::string& frame_id,
      const rclcpp::Time& stamp,
      int& marker_id,
      double line_width,
      double speed_color_max) const {
      using Marker = visualization_msgs::msg::Marker;
      using MarkerArray = visualization_msgs::msg::MarkerArray;
      using Point = geometry_msgs::msg::Point;
      using Point32 = geometry_msgs::msg::Point32;
      using PolygonStamped = geometry_msgs::msg::PolygonStamped;

      MarkerArray out;

      // 1) left boundary
      {
        auto pts = leftBound();
        if (!pts.empty()) {
          Marker m;
          m.header.frame_id = frame_id;
          m.header.stamp = stamp;
          m.ns = "lanelet_left";
          m.id = marker_id++;
          m.type = Marker::LINE_STRIP;
          m.action = Marker::ADD;
          m.scale.x = line_width;
          m.color.r = 0.863f;
          m.color.g = 0.902f;
          m.color.b = 0.459f;
          m.color.a = 1.0f;

          for (auto& p3 : pts) {
            Point p;
            p.x = p3.x();
            p.y = p3.y();
            p.z = p3.z();
            m.points.push_back(p);
          }
          out.markers.push_back(std::move(m));
        }
      }

      // 2) right boundary
      {
        auto pts = rightBound();
        if (!pts.empty()) {
          Marker m;
          m.header.frame_id = frame_id;
          m.header.stamp = stamp;
          m.ns = "lanelet_right";
          m.id = marker_id++;
          m.type = Marker::LINE_STRIP;
          m.action = Marker::ADD;
          m.scale.x = line_width;
          m.color.r = 0.898f;
          m.color.g = 0.451f;
          m.color.b = 0.451f;
          m.color.a = 1.0f;

          for (auto& p3 : pts) {
            Point p;
            p.x = p3.x();
            p.y = p3.y();
            p.z = p3.z();
            m.points.push_back(p);
          }
          out.markers.push_back(std::move(m));
        }
      }

      // 3) build polygon (left + reverse(right))
      geometry_msgs::msg::PolygonStamped poly;
      poly.header.frame_id = frame_id;
      poly.header.stamp = stamp;

      // add left
      for (auto& p3 : leftBound()) {
        geometry_msgs::msg::Point32 p;
        p.x = p3.x();
        p.y = p3.y();
        p.z = p3.z();
        poly.polygon.points.push_back(p);
      }

      // add right in reverse without rbegin/rend():
      {
        auto pts = rightBound();
        for (size_t idx = pts.size(); idx-- > 0;) {
          const auto& p3 = pts[idx];
          geometry_msgs::msg::Point32 p;
          p.x = p3.x();
          p.y = p3.y();
          p.z = p3.z();
          poly.polygon.points.push_back(p);
        }
      }



      // fill polygon with speed‐based color
      {
        // define your color anchors
        const float g_r = 127.0f / 255.0f, g_g = 255.0f / 255.0f, g_b = 187.0f / 255.0f;
        const float b_r = 0.0f / 255.0f, b_g = 136.0f / 255.0f, b_b = 204.0f / 255.0f;
        const float r_r = 231.0f / 255.0f, r_g = 54.0f / 255.0f, r_b = 102.0f / 255.0f;

        double sp = speedLimit();
        float rr, gg, bb;

        if (sp < 0.0) {
          rr = g_r;
          gg = g_g;
          bb = g_b;
        }
        else if (sp > speed_color_max) {
          rr = r_r;
          gg = r_g;
          bb = r_b;
        }
        else if (sp <= speed_color_max / 2.0) {
          rr = mapval(sp, 0.0, speed_color_max / 2.0, g_r, b_r);
          gg = mapval(sp, 0.0, speed_color_max / 2.0, g_g, b_g);
          bb = mapval(sp, 0.0, speed_color_max / 2.0, g_b, b_b);
        }
        else {
          rr = mapval(sp, speed_color_max / 2.0, speed_color_max, b_r, r_r);
          gg = mapval(sp, speed_color_max / 2.0, speed_color_max, b_g, r_g);
          bb = mapval(sp, speed_color_max / 2.0, speed_color_max, b_b, r_b);
        }

        Marker fill = triangulatePolygon(poly, rr, gg, bb, 0.1f, marker_id++);
        out.markers.push_back(std::move(fill));
      }

      if (oneWay()) {
        auto L = leftBound(), R = rightBound();
        if (L.size() > 1 && R.size() > 1) {
          // midpoint
          geometry_msgs::msg::Point mid;
          mid.x = (L[0].x() + R[0].x()) / 2;
          mid.y = (L[0].y() + R[0].y()) / 2;
          mid.z = 0.0;

          // direction vector
          double dx = ((L[1].x() + R[1].x()) / 2) - mid.x;
          double dy = ((L[1].y() + R[1].y()) / 2) - mid.y;
          double len = std::hypot(dx, dy);
          if (len > 1e-6) {
            dx /= len;
            dy /= len;
          }
          dx = -dx;
          dy = -dy; // reverse

          double sz = 1.0;
          double px = -dy, py = dx;

          // default‐construct and then assign
          geometry_msgs::msg::Point p1, p2, p3;
          p1.x = mid.x;
          p1.y = mid.y;
          p1.z = 0.0;
          p2.x = mid.x + sz * dx + sz * px;
          p2.y = mid.y + sz * dy + sz * py;
          p2.z = 0.0;
          p3.x = mid.x + sz * dx - sz * px;
          p3.y = mid.y + sz * dy - sz * py;
          p3.z = 0.0;

          Marker arr;
          arr.header.frame_id = frame_id;
          arr.header.stamp = stamp;
          arr.ns = "lanelet_one_way";
          arr.id = marker_id++;
          arr.type = Marker::TRIANGLE_LIST;
          arr.action = Marker::ADD;
          arr.scale.x = arr.scale.y = arr.scale.z = 1.0f;
          arr.color.r = arr.color.g = arr.color.b = arr.color.a = 1.0f;

          // push_back instead of brace-init
          arr.points.push_back(p1);
          arr.points.push_back(p2);
          arr.points.push_back(p3);

          out.markers.push_back(std::move(arr));
        }
      }

      return out;
    }

  private:
    const lanelet::ConstLanelet* _llt;
    static float mapval(float value, float istart, float iend, float ostart, float oend) {
      return ostart + (oend - ostart) * ((value - istart) / (iend - istart));
    }
  };

} // namespace laneviz
