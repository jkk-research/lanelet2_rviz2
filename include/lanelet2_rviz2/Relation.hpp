#ifndef LANELET2_RVIZ2_RELATION_HPP
#define LANELET2_RVIZ2_RELATION_HPP

#include <vector>
#include <string>
#include <map>
#include <cmath>

// ROS2 messages
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

// Include your own headers
#include "node.hpp"

#include "lanelet2_rviz2/triangulate_polygon.hpp" // Include your triangulation function

// Forward declaration for ROS time (you might include rclcpp if needed)
#include <rclcpp/rclcpp.hpp>

class OSMVisualizer;

namespace osm {

// Forward declaration for Way so that pointer types can be used.
class Way;

/**
 * @brief Abstract base class for relations.
 */
class Relation  {
public:
    struct Member {
        std::string type;
        std::string role;
        I64 ref;
    };

    Relation() = default;
    Relation(I64 id) : id_(id) {}
    Relation(const Relation&) = default;
    Relation(Relation&&) = default;
    Relation& operator=(const Relation&) = default;
    Relation& operator=(Relation&&) = default;
    virtual ~Relation() = default;

    // NOTE: The base draw() was originally a void function. In this refactoring,
    // we assume that only LaneletRelation uses the visualization message.
    virtual visualization_msgs::msg::MarkerArray draw(const std::string &frame_id,
                                                    const rclcpp::Time &stamp,
                                                    int &marker_id,
                                                    double line_width,
                                                    double speed_color_max) const = 0;

        // Set the vector of members parsed from the OSM file
    void set_members(const std::vector<Member>& members) {
        members_ = members;
    }
    
    // (Optional) Getter for the members vector
    const std::vector<Member>& members() const {
        return members_;
    }

    // Add a tag key/value pair to this relation
    void add_tag(const std::string &key, const std::string &value) {
        tags_[key] = value;
    }
    
    // (Optional) Getter for tags
    const std::map<std::string, std::string>& tags() const {
        return tags_;
    }
    
    // (Optional) Helper to get a member by role
    osm::Way* getMemberByRole(const std::string &role, const std::map<I64, osm::Way*>& ways) const {
        for (const auto &m : members_) {
            if (m.role == role && ways.count(m.ref)) {
                return ways.at(m.ref);
            }
        }
        return nullptr;
    }

protected:
    I64 id_{0};
    std::string type_;
    std::string subtype_;
    std::vector<Member> members_;
    std::map<std::string, std::string> tags_;
};

/**
 * @brief Class representing a lanelet relation.
 *
 * This class not only contains lanelet–specific attributes but now also
 * implements drawing of its own visualization markers.
 */
class LaneletRelation : public Relation {
public:
    LaneletRelation(I64 id) : Relation(id) {}
    LaneletRelation(const LaneletRelation&) = default;
    LaneletRelation(LaneletRelation&&) = default;
    LaneletRelation& operator=(const LaneletRelation&) = default;
    LaneletRelation& operator=(LaneletRelation&&) = default;
    virtual ~LaneletRelation() = default;

    // Lanelet–specific getters.
    Way* left() const { return left_; }
    Way* right() const { return right_; }
    I64 speed_limit() const { return speed_limit_; }
    std::string location() const { return location_; }
    bool one_way() const { return one_way_; }
    std::string turn_direction() const { return turn_direction_; }

    /**
     * @brief Create visualization markers for this lanelet relation.
     *
     * @param frame_id The coordinate frame.
     * @param stamp The current time stamp.
     * @param marker_id A counter for unique marker IDs (which will be incremented).
     * @param line_width The desired line width for drawing.
     * @param speed_color_max The maximum speed used for color mapping.
     * @return MarkerArray message with all markers for the relation.
     */
    visualization_msgs::msg::MarkerArray draw(const std::string &frame_id,
                                                const rclcpp::Time &stamp,
                                                int &marker_id,
                                                double line_width,
                                                double speed_color_max) const override
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // --- Create left way marker ---
        if (left_) {
            visualization_msgs::msg::Marker left_marker;
            left_marker.header.frame_id = frame_id;
            left_marker.header.stamp = stamp;
            left_marker.ns = "osm_relations_left";
            left_marker.id = marker_id++;
            left_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            left_marker.action = visualization_msgs::msg::Marker::ADD;
            // Set color for left way (lime)
            left_marker.color.r = 0.863;
            left_marker.color.g = 0.902;
            left_marker.color.b = 0.459;
            left_marker.color.a = 1.0;
            left_marker.scale.x = line_width;

            // Add points from left way nodes.
            for (const osm::Node *node : left_->nodes()) {
                geometry_msgs::msg::Point point;
                point.x = node->local_x();
                point.y = node->local_y();
                point.z = node->ele();
                left_marker.points.push_back(point);
            }
            marker_array.markers.push_back(left_marker);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("LaneletRelation"), "Relation (ID: %lld) has no left member", id_);
        }

        // --- Create right way marker ---
        if (right_) {
            visualization_msgs::msg::Marker right_marker;
            right_marker.header.frame_id = frame_id;
            right_marker.header.stamp = stamp;
            right_marker.ns = "osm_relations_right";
            right_marker.id = marker_id++;
            right_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            right_marker.action = visualization_msgs::msg::Marker::ADD;
            // Set color for right way (red)
            right_marker.color.r = 0.898;
            right_marker.color.g = 0.451;
            right_marker.color.b = 0.451;
            right_marker.color.a = 1.0;
            right_marker.scale.x = line_width;

            // Add points from right way nodes.
            for (const osm::Node *node : right_->nodes()) {
                geometry_msgs::msg::Point point;
                point.x = node->local_x();
                point.y = node->local_y();
                point.z = node->ele();
                right_marker.points.push_back(point);
            }
            marker_array.markers.push_back(right_marker);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("LaneletRelation"), "Relation (ID: %lld) has no right member", id_);
        }

        // --- Create polygon for triangulation (speed marker) ---
        geometry_msgs::msg::PolygonStamped left_right_polygon;
        left_right_polygon.header.frame_id = frame_id;
        left_right_polygon.header.stamp = stamp;

        if (left_) {
            for (const osm::Node *node : left_->nodes()) {
                geometry_msgs::msg::Point32 point;
                point.x = node->local_x();
                point.y = node->local_y();
                point.z = node->ele();
                left_right_polygon.polygon.points.push_back(point);
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("LaneletRelation"), "Relation (ID: %lld) has no left member for polygon", id_);
        }
        if (right_) {
            // Add right way nodes in reverse order.
            for (auto it = right_->nodes().rbegin(); it != right_->nodes().rend(); ++it) {
                const osm::Node *node = *it;
                geometry_msgs::msg::Point32 point;
                point.x = node->local_x();
                point.y = node->local_y();
                point.z = node->ele();
                left_right_polygon.polygon.points.push_back(point);
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("LaneletRelation"), "Relation (ID: %lld) has no right member for polygon", id_);
        }

        // --- Set up gradient color based on speed limit ---
        float green_r = 127.0f / 255.0f, green_g = 255.0f / 255.0f, green_b = 187.0f / 255.0f;
        float red_r = 231.0f / 255.0f, red_g = 54.0f / 255.0f, red_b = 102.0f / 255.0f;
        float blue_r = 0.0f / 255.0f, blue_g = 136.0f / 255.0f, blue_b = 204.0f / 255.0f;
        double r_color, g_color, b_color;

        if (speed_limit() < 0) {
            r_color = green_r; g_color = green_g; b_color = green_b;
        } else if (speed_limit() > speed_color_max) {
            r_color = red_r; g_color = red_g; b_color = red_b;
        } else if (speed_limit() > 0 && speed_limit() < speed_color_max / 2) {
            float c0 = mapval(speed_limit(), 0, speed_color_max / 2, green_r, blue_r);
            float c1 = mapval(speed_limit(), 0, speed_color_max / 2, green_g, blue_g);
            float c2 = mapval(speed_limit(), 0, speed_color_max / 2, green_b, blue_b);
            r_color = c0; g_color = c1; b_color = c2;
        } else {
            float c0 = mapval(speed_limit(), speed_color_max / 2, speed_color_max, blue_r, red_r);
            float c1 = mapval(speed_limit(), speed_color_max / 2, speed_color_max, blue_g, red_g);
            float c2 = mapval(speed_limit(), speed_color_max / 2, speed_color_max, blue_b, red_b);
            r_color = c0; g_color = c1; b_color = c2;
        }

        // --- Triangulate the polygon --- 
        // Assume triangulatePolygon is a helper function that creates a Marker from a polygon.
        visualization_msgs::msg::Marker triangle_marker =
            triangulatePolygon(left_right_polygon, r_color, g_color, b_color, 0.1, marker_id++);
        marker_array.markers.push_back(triangle_marker);

        // --- Create one–way arrow marker ---
        visualization_msgs::msg::Marker one_way_arrow;
        one_way_arrow.header.frame_id = frame_id;
        one_way_arrow.header.stamp = stamp;
        one_way_arrow.ns = "osm_one_way_arrows";
        one_way_arrow.id = marker_id++;
        one_way_arrow.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        one_way_arrow.action = visualization_msgs::msg::Marker::ADD;
        // Set arrow color to white.
        one_way_arrow.color.r = 1.0;
        one_way_arrow.color.g = 1.0;
        one_way_arrow.color.b = 1.0;
        one_way_arrow.color.a = 1.0;
        one_way_arrow.scale.x = 1.0;
        one_way_arrow.scale.y = 1.0;
        one_way_arrow.scale.z = 1.0;

        if (!left_ || left_->nodes().empty() || !right_ || right_->nodes().empty()) {
            RCLCPP_WARN(rclcpp::get_logger("LaneletRelation"), "Relation (ID: %lld) missing left or right member for arrow", id_);
        } else {
            const osm::Node *left_first = left_->nodes().front();
            const osm::Node *right_first = right_->nodes().front();
            geometry_msgs::msg::Point midpoint;
            midpoint.x = (left_first->local_x() + right_first->local_x()) / 2;
            midpoint.y = (left_first->local_y() + right_first->local_y()) / 2;
            midpoint.z = 0.0;

            if (left_->nodes().size() > 1 && right_->nodes().size() > 1) {
                const osm::Node *left_second = left_->nodes()[1];
                const osm::Node *right_second = right_->nodes()[1];
                double dir_x = ((left_second->local_x() + right_second->local_x()) / 2) - midpoint.x;
                double dir_y = ((left_second->local_y() + right_second->local_y()) / 2) - midpoint.y;
                double length = std::sqrt(dir_x * dir_x + dir_y * dir_y);
                if (length != 0) {
                    dir_x /= length;
                    dir_y /= length;
                }
                // Reverse the direction.
                dir_x = -dir_x;
                dir_y = -dir_y;
                geometry_msgs::msg::Point p1, p2, p3;
                double arrow_size = 1.0;
                p1.x = midpoint.x;
                p1.y = midpoint.y;
                p1.z = 0.0;
                double perp_x = -dir_y;
                double perp_y = dir_x;
                p2.x = midpoint.x + arrow_size * dir_x + arrow_size * perp_x;
                p2.y = midpoint.y + arrow_size * dir_y + arrow_size * perp_y;
                p2.z = 0.0;
                p3.x = midpoint.x + arrow_size * dir_x - arrow_size * perp_x;
                p3.y = midpoint.y + arrow_size * dir_y - arrow_size * perp_y;
                p3.z = 0.0;

                one_way_arrow.points.push_back(p1);
                one_way_arrow.points.push_back(p2);
                one_way_arrow.points.push_back(p3);

                marker_array.markers.push_back(one_way_arrow);
            } else {
                RCLCPP_WARN(rclcpp::get_logger("LaneletRelation"), "Relation (ID: %lld) has insufficient nodes for arrow", id_);
            }
        }

        return marker_array;
    }

private:
    osm::Way* left_{nullptr};   ///< Pointer to the left Way.
    osm::Way* right_{nullptr};  ///< Pointer to the right Way.
    I64 speed_limit_{0};         ///< Speed limit for the lanelet.
    std::string location_;       ///< Location descriptor.
    bool one_way_{false};        ///< Whether the lane is one-way.
    std::string turn_direction_; ///< Turn direction string.

    // --- Helper functions ---

    /**
     * @brief Maps a value from one range to another.
     *
     * @param value The input value.
     * @param istart The lower bound of the input range.
     * @param iend The upper bound of the input range.
     * @param ostart The lower bound of the output range.
     * @param oend The upper bound of the output range.
     * @return The mapped value.
     */
    static float mapval(float value, float istart, float iend, float ostart, float oend) {
        return ostart + (oend - ostart) * ((value - istart) / (iend - istart));
    }


};

} // namespace osm

#endif // LANELET2_RVIZ2_RELATION_HPP
