#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "lanelet2_rviz2/rapidxml.hpp"
#include "lanelet2_rviz2/rapidxml_utils.hpp"
#include "lanelet2_rviz2/node.hpp"
#include "lanelet2_rviz2/way.hpp"
#include "lanelet2_rviz2/relation.hpp"
#include "lanelet2_rviz2/earcut.hpp"
#include <map>
#include <vector>
#include <numeric>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;
using geometry_msgs::msg::PolygonStamped;
using geometry_msgs::msg::Point;
using visualization_msgs::msg::Marker;

class OSMVisualizer : public rclcpp::Node {
public:
    OSMVisualizer() : Node("osm_visualizer")
    {
        this->declare_parameter<double>("line_width", 0.8);
        this->declare_parameter<std::string>("frame_id", "map_gyor_0");
        this->declare_parameter<std::string>("osm_filename", "");
        this->declare_parameter<bool>("center_map", false);
        this->declare_parameter<double>("speed_color_max", 90.0);
        this->get_parameter("line_width", line_width_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("osm_filename", filename_);
        this->get_parameter("center_map", center_map_);
        this->get_parameter("speed_color_max", speed_color_max_);

        // Register parameter callback
        on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&OSMVisualizer::parametersCallback, this, _1));

        // marker_array_ways_pub_(this->create_publisher<visualization_msgs::msg::MarkerArray>("osm_markers", 10));
        marker_array_relations_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("osm_relations", 10);
        marker_array_speed_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("osm_speed", 10);
        marker_array_one_way_arrows_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("osm_one_way_arrows", 10);


        // Load and parse the OSM file
        if (!filename_.empty()) {
            try {
                parseOSMFile(filename_.c_str());
                if (center_map_) centerCoordinates();
                data_loaded_ = true;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse OSM file: %s", e.what());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "OSM filename parameter is empty.");
        }

        // Create a timer to republish markers every second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&OSMVisualizer::publishMarkers, this));
        RCLCPP_INFO(this->get_logger(), "OSM Visualizer node started.");
    }
private:
    std::map<int, osm::Node*> nodes_;
    std::map<int, osm::Way*> ways_;
    std::map<int, osm::Relation*> relations_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_ways_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_relations_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_speed_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_one_way_arrows_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
    double center_x_ = 0.0; // Map center X coordinate
    double center_y_ = 0.0; // Map center Y coordinate
    bool data_loaded_ = false; // Flag to indicate if data has been loaded
    double line_width_ = 0.8; // Line width for visualization
    std::string frame_id_ = "map_gyor_0"; // Frame ID for visualization
    std::string filename_; // OSM filename parameter
    bool center_map_ = false; // Center map if true (could be useful for debugging)
    double speed_color_max_ = 90.0; // Maximum speed limit for color scaling

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto& param : parameters) {
            RCLCPP_INFO(this->get_logger(), "Parameter updated: %s = %s",
                        param.get_name().c_str(), param.value_to_string().c_str());

            if (param.get_name() == "line_width") {
                line_width_ = param.as_double();
            } else if (param.get_name() == "frame_id") {
                frame_id_ = param.as_string();
            } else if (param.get_name() == "osm_filename") {
                filename_ = param.as_string();
                try {
                    parseOSMFile(filename_.c_str());
                    if (center_map_) centerCoordinates();
                    data_loaded_ = true;
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to parse OSM file: %s", e.what());
                    result.successful = false;
                    result.reason = "Failed to parse OSM file";
                }
            } else if (param.get_name() == "center_map") {
                center_map_ = param.as_bool();
                if (center_map_) {
                    centerCoordinates();
                }
            } else if (param.get_name() == "speed_color_max") {
                speed_color_max_ = param.as_double();
            }
        }

        return result;
    }

    void parseOSMFile(const char* filename) {
        // Clear previously loaded data
        clearData();

        rapidxml::file<> xmlFile(filename);
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());

        rapidxml::xml_node<>* root = doc.first_node("osm");

        // Parse nodes
        for (rapidxml::xml_node<>* node = root->first_node("node"); node; node = node->next_sibling("node")) {
            int id = std::stoi(node->first_attribute("id")->value());
            double lat = std::stod(node->first_attribute("lat")->value());
            double lon = std::stod(node->first_attribute("lon")->value());
            double local_x = 0, local_y = 0, ele = 0;

            // Parse tags
            for (rapidxml::xml_node<>* tag = node->first_node("tag"); tag; tag = tag->next_sibling("tag")) {
                std::string key = tag->first_attribute("k")->value();
                double value = std::stod(tag->first_attribute("v")->value());
                if (key == "local_x") local_x = value;
                else if (key == "local_y") local_y = value;
                else if (key == "ele") ele = value;
            }

            osm::Node* newNode = new osm::Node(id, lat, lon, local_x, local_y, ele);
            nodes_[id] = newNode;
        }

        // Parse ways
        for (rapidxml::xml_node<>* wayNode = root->first_node("way"); wayNode; wayNode = wayNode->next_sibling("way")) {
            int id = std::stoi(wayNode->first_attribute("id")->value());
            osm::Way* way = new osm::Way(id);

            // Parse nd references
            for (rapidxml::xml_node<>* nd = wayNode->first_node("nd"); nd; nd = nd->next_sibling("nd")) {
                int ref = std::stoi(nd->first_attribute("ref")->value());
                if (nodes_.count(ref)) {
                    way->add_node(nodes_[ref]);
                }
            }

            ways_[id] = way;
        }

        // Parse relations
        for (rapidxml::xml_node<>* relationNode = root->first_node("relation"); relationNode; relationNode = relationNode->next_sibling("relation")) {
            int id = std::stoi(relationNode->first_attribute("id")->value());
            osm::Way* left = nullptr;
            osm::Way* right = nullptr;
            std::string type, subtype, location;
            int speed_limit = 0;
            bool one_way = false;

            // Parse members
            for (rapidxml::xml_node<>* member = relationNode->first_node("member"); member; member = member->next_sibling("member")) {
                std::string role = member->first_attribute("role")->value();
                int ref = std::stoi(member->first_attribute("ref")->value());
                if (role == "left") {
                    if (ways_.count(ref)) {
                        left = ways_[ref];
                    }
                } else if (role == "right") {
                    if (ways_.count(ref)) {
                        right = ways_[ref];
                    }
                }
            }

            // Parse tags
            for (rapidxml::xml_node<>* tag = relationNode->first_node("tag"); tag; tag = tag->next_sibling("tag")) {
                std::string key = tag->first_attribute("k")->value();
                std::string value = tag->first_attribute("v")->value();
                if (key == "type") type = value;
                else if (key == "subtype") subtype = value;
                else if (key == "speed_limit") speed_limit = std::stoi(value);
                else if (key == "location") location = value;
                else if (key == "one_way") one_way = value == "yes";
            }

            osm::Relation* relation = new osm::Relation(id, left, right, type, subtype, speed_limit, location, one_way);
            relations_[id] = relation;
        }
    }

    Marker triangulatePolygon(const PolygonStamped &polygon, double r=1.0, double g=1.0, double b=1.0, double a=1.0, int id=0) {
        Marker triangle_marker;

        if (polygon.polygon.points.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Polygon has less than 3 points; cannot triangulate.");
            return triangle_marker;
        }

        // Set up the marker properties
        triangle_marker.header = polygon.header;
        triangle_marker.ns = "triangulated_polygon";
        triangle_marker.id = id;
        triangle_marker.type = Marker::TRIANGLE_LIST;
        triangle_marker.action = Marker::ADD;

        triangle_marker.scale.x = 1.0;
        triangle_marker.scale.y = 1.0;
        triangle_marker.scale.z = 1.0;

        // Set a default color (optional)
        triangle_marker.color.r = r;
        triangle_marker.color.g = g;
        triangle_marker.color.b = b;
        triangle_marker.color.a = a;

        // Prepare input for Earcut
        using Coord = double;
        std::vector<std::vector<std::array<Coord, 2>>> polygon_coords;
        std::vector<std::array<Coord, 2>> ring;
        for (const auto &point : polygon.polygon.points) {
            ring.push_back({point.x, point.y});
        }
        polygon_coords.push_back(ring);

        // Perform triangulation using Earcut
        std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(polygon_coords);

        // Convert Earcut output into Marker points
        const auto &vertices = polygon_coords[0];
        for (size_t i = 0; i < indices.size(); i += 3) {
            // For each triangle, add the three vertices to the marker
            for (int j = 0; j < 3; ++j) {
                Point p;
                p.x = vertices[indices[i + j]][0];
                p.y = vertices[indices[i + j]][1];
                p.z = 0.0; // Assuming 2D polygon in XY plane
                triangle_marker.points.push_back(p);
            }
        }

        return triangle_marker;
    }

    void centerCoordinates() {
        // Calculate the centroid of the map
        double sum_x = 0.0;
        double sum_y = 0.0;
        size_t node_count = nodes_.size();

        for (const auto& node_pair : nodes_) {
            osm::Node* node = node_pair.second;
            sum_x += node->local_x();
            sum_y += node->local_y();
        }

        if (node_count > 0) {
            center_x_ = sum_x / node_count;
            center_y_ = sum_y / node_count;
        }

        // Translate all nodes to center the map
        for (auto& node_pair : nodes_) {
            osm::Node* node = node_pair.second;
            node->set_local_x(node->local_x() - center_x_);
            node->set_local_y(node->local_y() - center_y_);
        }

        RCLCPP_INFO(this->get_logger(), "Map centered at (%.2f, %.2f)", center_x_, center_y_);
    }

    float mapval(float x, float in_min, float in_max, float out_min, float out_max) const{
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    void publishMarkers() {
        if (!data_loaded_) {
            return; // Only publish if data is loaded
        }

        // visualization_msgs::msg::MarkerArray marker_array_ways;
        visualization_msgs::msg::MarkerArray marker_array_relations;
        visualization_msgs::msg::MarkerArray marker_array_speed;
        visualization_msgs::msg::MarkerArray marker_array_one_way_arrows;

        int marker_id = 0;

        // visualize the ways in relations
        for (const auto& relation_pair : relations_) {
            const osm::Relation& relation = *relation_pair.second;

            // Marker for left way
            visualization_msgs::msg::Marker left_marker;
            left_marker.header.frame_id = frame_id_;
            left_marker.header.stamp = this->now();
            left_marker.ns = "osm_relations_left";
            left_marker.id = marker_id++;
            left_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            left_marker.action = visualization_msgs::msg::Marker::ADD;

            // Set color for left way (lime) md_lime_300 #DCE775 0.863, 0.902, 0.459 https://github.com/jkk-research/colors
            left_marker.color.r = 0.863;
            left_marker.color.g = 0.902;
            left_marker.color.b = 0.459;
            left_marker.color.a = 1.0;

            // Line width
            left_marker.scale.x = line_width_;

            // Add points from left way nodes
            for (const osm::Node* node : relation.left()->nodes()) {
                geometry_msgs::msg::Point point;
                point.x = node->local_x();
                point.y = node->local_y();
                point.z = node->ele(); 
                left_marker.points.push_back(point);
            }

            marker_array_relations.markers.push_back(left_marker);

            // Marker for right way
            visualization_msgs::msg::Marker right_marker;
            right_marker.header.frame_id = frame_id_;
            right_marker.header.stamp = this->now();
            right_marker.ns = "osm_relations_right";
            right_marker.id = marker_id++;
            right_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            right_marker.action = visualization_msgs::msg::Marker::ADD;

            // Set color for right way (red) md_red_300 #E57373 0.898, 0.451, 0.451 https://github.com/jkk-research/colors 
            right_marker.color.r = 0.898;
            right_marker.color.g = 0.451;
            right_marker.color.b = 0.451;
            right_marker.color.a = 1.0;

            // Line width
            right_marker.scale.x = line_width_;

            // Add points from right way nodes
            for (const osm::Node* node : relation.right()->nodes()) {
                geometry_msgs::msg::Point point;
                point.x = node->local_x();
                point.y = node->local_y();
                point.z = node->ele(); 
                right_marker.points.push_back(point);
            }

            marker_array_relations.markers.push_back(right_marker);


            // make polygons from the left and right ways
            PolygonStamped left_right_polygon;
            left_right_polygon.header.frame_id = frame_id_;
            left_right_polygon.header.stamp = this->now();
            for (const osm::Node* node : relation.left()->nodes()) {
                geometry_msgs::msg::Point32 point;
                point.x = node->local_x();
                point.y = node->local_y();
                point.z = node->ele(); 
                left_right_polygon.polygon.points.push_back(point);
            }
            // the right way is traversed in reverse order, so they are connected properly
            for (auto it = relation.right()->nodes().rbegin(); it != relation.right()->nodes().rend(); ++it) {
                const osm::Node* node = *it;
                geometry_msgs::msg::Point32 point;
                point.x = node->local_x();
                point.y = node->local_y();
                point.z = node->ele();
                left_right_polygon.polygon.points.push_back(point);
            }

            // gradient color based on speed limit
            double r, g, b;
            // green-blue-red gradient: https://coolors.co/gradient-maker/7fffbb-0088cc-e73666
            float green_r = 127. / 255., green_g = 255. / 255., green_b = 187. / 255.; // #7FFFBB 127, 255, 187
            float red_r = 231. / 255., red_g = 54. / 255., red_b = 102. / 255.;        // #E73666 231, 54, 102
            float blue_r = 0. / 255., blue_g = 136. / 255., blue_b = 204. / 255.;      // #0088CC 0, 136, 204
            if (relation.speed_limit() < 0) {
                r = green_r;
                g = green_g;
                b = green_b;
            }
            else if (relation.speed_limit() > speed_color_max_) {
                r = red_r;
                g = red_g;
                b = red_b;
            }
            else if (0 < relation.speed_limit() && relation.speed_limit() < speed_color_max_ / 2) {
                float c0 = mapval(relation.speed_limit(), 0, speed_color_max_ / 2, green_r, blue_r);
                float c1 = mapval(relation.speed_limit(), 0, speed_color_max_ / 2, green_g, blue_g);
                float c2 = mapval(relation.speed_limit(), 0, speed_color_max_ / 2, green_b, blue_b);
                r = c0;
                g = c1;
                b = c2;
            }
            else {
                float c0 = mapval(relation.speed_limit(), speed_color_max_ / 2, speed_color_max_, blue_r, red_r);
                float c1 = mapval(relation.speed_limit(), speed_color_max_ / 2, speed_color_max_, blue_g, red_g);
                float c2 = mapval(relation.speed_limit(), speed_color_max_ / 2, speed_color_max_, blue_b, red_b);
                r = c0;
                g = c1;
                b = c2;
            }

            // triangulate the polygon
            Marker triangle_marker = triangulatePolygon(left_right_polygon, r, g, b, 0.1, marker_id++);
            marker_array_speed.markers.push_back(triangle_marker);

            // visualize arrows that show the one-way direction of each lane
            visualization_msgs::msg::Marker one_way_arrow;
            one_way_arrow.header.frame_id = frame_id_;
            one_way_arrow.header.stamp = this->now();
            one_way_arrow.ns = "osm_one_way_arrows";
            one_way_arrow.id = marker_id++;
            one_way_arrow.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            one_way_arrow.action = visualization_msgs::msg::Marker::ADD;

            // Get the middle point of the first points of the left and right ways
            const osm::Node* left_first = relation.left()->nodes().front();
            const osm::Node* right_first = relation.right()->nodes().front();
            geometry_msgs::msg::Point midpoint;
            midpoint.x = (left_first->local_x() + right_first->local_x()) / 2;
            midpoint.y = (left_first->local_y() + right_first->local_y()) / 2;
            midpoint.z = 0.0;

            // Set color for one way arrow white
            one_way_arrow.color.r = 1.0;
            one_way_arrow.color.g = 1.0;
            one_way_arrow.color.b = 1.0;
            one_way_arrow.color.a = 1.0; // Set alpha to 1.0 to make it visible

            // Scale
            one_way_arrow.scale.x = 1.0;
            one_way_arrow.scale.y = 1.0;
            one_way_arrow.scale.z = 1.0;

            // Calculate the direction vector of the lane based on the first and second points
            const osm::Node* left_second = relation.left()->nodes()[1];
            const osm::Node* right_second = relation.right()->nodes()[1];
            double dir_x = ((left_second->local_x() + right_second->local_x()) / 2) - midpoint.x;
            double dir_y = ((left_second->local_y() + right_second->local_y()) / 2) - midpoint.y;
            double length = std::sqrt(dir_x * dir_x + dir_y * dir_y);
            dir_x /= length;
            dir_y /= length;
            dir_x = -dir_x;
            dir_y = -dir_y;

            // Define the points for the arrow triangle
            geometry_msgs::msg::Point p1, p2, p3;
            double arrow_size = 1.0; // Adjust this value to control the arrow size

            // Pointing direction (90-degree angle at p1)
            p1.x = midpoint.x;
            p1.y = midpoint.y;
            p1.z = 0.0;

            // Calculate the perpendicular vector to the direction vector
            double perp_x = -dir_y;
            double perp_y = dir_x;

            // Define the other two points of the triangle
            p2.x = midpoint.x + arrow_size * dir_x + arrow_size * perp_x;
            p2.y = midpoint.y + arrow_size * dir_y + arrow_size * perp_y;
            p2.z = 0.0;

            p3.x = midpoint.x + arrow_size * dir_x - arrow_size * perp_x;
            p3.y = midpoint.y + arrow_size * dir_y - arrow_size * perp_y;
            p3.z = 0.0;

            // Add the points to form the triangle
            one_way_arrow.points.push_back(p1);
            one_way_arrow.points.push_back(p2);
            one_way_arrow.points.push_back(p3);

            // only push the marker if the distance from the previous one is greater than 10 (to avoid overlapping arrows, if a way is very short)
            if (marker_array_one_way_arrows.markers.empty()) {
                marker_array_one_way_arrows.markers.push_back(one_way_arrow);
            } else {
                const auto& prev_arrow = marker_array_one_way_arrows.markers.back();
                double prev_x = prev_arrow.points[0].x;
                double prev_y = prev_arrow.points[0].y;
                double dist = std::sqrt((midpoint.x - prev_x) * (midpoint.x - prev_x) + (midpoint.y - prev_y) * (midpoint.y - prev_y));
                if (dist > 4) {
                    marker_array_one_way_arrows.markers.push_back(one_way_arrow);
                }
            }

        }

        // remove any arrows that are closer than 2 meters to each other (they might not be next to each other in the array)
        for (size_t i = 0; i < marker_array_one_way_arrows.markers.size(); ++i) {
            for (size_t j = i + 1; j < marker_array_one_way_arrows.markers.size(); ++j) {
                const auto& arrow1 = marker_array_one_way_arrows.markers[i];
                const auto& arrow2 = marker_array_one_way_arrows.markers[j];
                double dist = std::sqrt((arrow1.points[0].x - arrow2.points[0].x) * (arrow1.points[0].x - arrow2.points[0].x) + (arrow1.points[0].y - arrow2.points[0].y) * (arrow1.points[0].y - arrow2.points[0].y));
                if (dist < 2) {
                    marker_array_one_way_arrows.markers.erase(marker_array_one_way_arrows.markers.begin() + j);
                    --j;
                }
            }
        }

        marker_array_relations_pub_->publish(marker_array_relations);
        marker_array_speed_pub_->publish(marker_array_speed);
        marker_array_one_way_arrows_pub_->publish(marker_array_one_way_arrows);
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Published on frame: " << frame_id_);
    }

    void clearData() {
        for (auto& pair : nodes_) delete pair.second;
        nodes_.clear();

        for (auto& pair : ways_) delete pair.second;
        ways_.clear();

        for (auto& pair : relations_) delete pair.second;
        relations_.clear();
    }

    // ~OSMVisualizer() {
    //     clearData();
    // }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OSMVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
