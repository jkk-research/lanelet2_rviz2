#ifndef LANELET2_RVIZ2_TRIANGULATE_POLYGON_HPP
#define LANELET2_RVIZ2_TRIANGULATE_POLYGON_HPP

#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include "lanelet2_rviz2/earcut.hpp"


/**
 * @brief Triangulates a polygon to produce a Marker message.
 *
 * This is a stub for your own triangulation routine.
 *
 * @param polygon The input polygon.
 * @param r Color red component.
 * @param g Color green component.
 * @param b Color blue component.
 * @param a Color alpha component.
 * @param id Unique marker id.
 * @return A Marker message representing the triangulated polygon.
 */
inline visualization_msgs::msg::Marker triangulatePolygon(
    const geometry_msgs::msg::PolygonStamped &polygon,
    double r, double g, double b, double a, I64 id)
{
    visualization_msgs::msg::Marker triangle_marker;

    if (polygon.polygon.points.size() < 3)
    {
        RCLCPP_WARN(rclcpp::get_logger("triangulator"), "Polygon has less than 3 points; cannot triangulate.");
        return triangle_marker;
    }

    // Set up the marker properties.
    triangle_marker.header = polygon.header;
    triangle_marker.ns = "triangulated_polygon";
    triangle_marker.id = id;
    triangle_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    triangle_marker.action = visualization_msgs::msg::Marker::ADD;

    triangle_marker.scale.x = 1.0;
    triangle_marker.scale.y = 1.0;
    triangle_marker.scale.z = 1.0;

    triangle_marker.color.r = r;
    triangle_marker.color.g = g;
    triangle_marker.color.b = b;
    triangle_marker.color.a = a;

    // Prepare input for Earcut.
    using Coord = double;
    std::vector<std::vector<std::array<Coord, 2>>> polygon_coords;
    std::vector<std::array<Coord, 2>> ring;
    for (const auto &point : polygon.polygon.points)
    {
        ring.push_back({point.x, point.y});
    }
    polygon_coords.push_back(ring);

    // Perform triangulation.
    std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(polygon_coords);

    // Convert Earcut output into Marker points.
    const auto &vertices = polygon_coords[0];
    for (size_t i = 0; i < indices.size(); i += 3)
    {
        for (int j = 0; j < 3; ++j)
        {
            geometry_msgs::msg::Point p;
            p.x = vertices[indices[i + j]][0];
            p.y = vertices[indices[i + j]][1];
            p.z = 0.0; // Assuming 2D polygon in the XY plane.
            triangle_marker.points.push_back(p);
        }
    }

    return triangle_marker;
}

#endif // LANELET2_RVIZ2_TRIANGULATE_POLYGON_HPP