#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace laneviz {

  class ParkingSpace {
  public:
    // Construct from a 2-point lanelet2 linestring
    explicit ParkingSpace(const lanelet::ConstLineString3d& ls) noexcept
      : _ls(&ls) {
      // ensure it really has two points
      if (_ls->size() != 2) {
        RCLCPP_WARN(
          rclcpp::get_logger("ParkingSpace"),
          "ParkingSpace initialized with %zu points (expected 2)",
          _ls->size()
        );
      }
    }

    /// Draws the parking‐space as a single white line.
    ///
    /// @param frame_id   tf frame in which to draw
    /// @param stamp      ROS time stamp
    /// @param marker_id  in/out counter for unique marker IDs
    /// @param line_width width of the line
    visualization_msgs::msg::MarkerArray draw(
      const std::string& frame_id,
      const rclcpp::Time& stamp,
      int& marker_id,
      double                       line_width) const {
      using Marker = visualization_msgs::msg::Marker;
      using MarkerArray = visualization_msgs::msg::MarkerArray;
      using Point = geometry_msgs::msg::Point;

      MarkerArray out;

      const auto& pts = *_ls;
      if (pts.size() < 2) {
        // nothing to draw
        return out;
      }

      Marker m;
      m.header.frame_id = frame_id;
      m.header.stamp = stamp;
      m.ns = "parking_space";
      m.id = marker_id++;
      m.type = Marker::LINE_LIST;
      m.action = Marker::ADD;
      m.scale.x = line_width;
      // white color
      m.color.r = 1.0f;
      m.color.g = 1.0f;
      m.color.b = 1.0f;
      m.color.a = 1.0f;

      // draw exactly one segment
      Point p0, p1;
      p0.x = pts[0].x();
      p0.y = pts[0].y();
      p0.z = pts[0].z();
      p1.x = pts[1].x();
      p1.y = pts[1].y();
      p1.z = pts[1].z();

      m.points.push_back(p0);
      m.points.push_back(p1);

      out.markers.push_back(std::move(m));
      return out;
    }

  private:
    const lanelet::ConstLineString3d* _ls;
  };

} // namespace laneviz
