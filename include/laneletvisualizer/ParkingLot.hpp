#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include "triangulate_polygon.hpp"

namespace laneviz {

  class ParkingLot {
  public:
    explicit ParkingLot(const lanelet::ConstPolygon3d& poly) noexcept
      : _poly(&poly) {}

    lanelet::ConstPolygon3d polygon() const noexcept { return *_poly; }

    visualization_msgs::msg::MarkerArray draw(
      const std::string& frame_id,
      const rclcpp::Time& stamp,
      int& marker_id,
      double             line_width,
      float              r,
      float              g,
      float              b) const {
      using Marker = visualization_msgs::msg::Marker;
      using MarkerArray = visualization_msgs::msg::MarkerArray;
      using Point = geometry_msgs::msg::Point;
      using Point32 = geometry_msgs::msg::Point32;
      using PolygonStamped = geometry_msgs::msg::PolygonStamped;

      MarkerArray out;
      const auto pts = polygon();
      const size_t N = pts.size();
      if (N < 3) return out;  // need at least a triangle to fill

      //
      // 1) OUTLINE with LINE_LIST
      //
      {
        Marker line;
        line.header.frame_id = frame_id;
        line.header.stamp = stamp;
        line.ns = "parkinglot_outline";
        line.id = marker_id++;
        line.type = Marker::LINE_LIST;
        line.action = Marker::ADD;
        line.scale.x = line_width;
        line.color.r = line.color.g = line.color.b = 1.0f;
        line.color.a = 1.0f;

        // add each edge as a separate segment
        for (size_t i = 0; i < N; ++i) {
          const auto& A = pts[i];
          const auto& B = pts[(i + 1) % N];
          Point pA, pB;
          pA.x = A.x();  pA.y = A.y();  pA.z = A.z();
          pB.x = B.x();  pB.y = B.y();  pB.z = B.z();
          line.points.push_back(pA);
          line.points.push_back(pB);
        }

        out.markers.push_back(std::move(line));
      }

      //
      // 2) Fill via triangulatePolygon
      //
      {
        PolygonStamped poly;
        poly.header.frame_id = frame_id;
        poly.header.stamp = stamp;

        for (const auto& P : pts) {
          Point32 p;
          p.x = P.x();  p.y = P.y();  p.z = P.z();
          poly.polygon.points.push_back(p);
        }

        // give it a tiny extrusion so it actually shows up
        constexpr float EXTRUDE_Z = 0.05f;
        auto fill = triangulatePolygon(poly, r, g, b, EXTRUDE_Z, marker_id++);
        fill.ns = "parkinglot_fill";
        fill.color.a = 0.1f;
        out.markers.push_back(std::move(fill));
      }

      return out;
    }

  private:
    const lanelet::ConstPolygon3d* _poly;
  };

} // namespace laneviz
