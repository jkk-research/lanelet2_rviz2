#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "lanelet2_rviz2/rapidxml.hpp"
#include "lanelet2_rviz2/rapidxml_utils.hpp"
#include "lanelet2_rviz2/node.hpp"
#include "lanelet2_rviz2/way.hpp"
#include "lanelet2_rviz2/relation.hpp"
#include <map>
#include <vector>
#include <numeric>

using namespace std::chrono_literals;
using std::placeholders::_1;

class OSMVisualizer : public rclcpp::Node {
public:
    OSMVisualizer() : Node("osm_visualizer")
    {
        this->declare_parameter<double>("line_width", 0.8);
        this->declare_parameter<std::string>("frame_id", "map_gyor_0");
        this->declare_parameter<std::string>("osm_filename", "");
        this->declare_parameter<bool>("center_map", false);
        this->get_parameter("line_width", line_width_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("osm_filename", filename_);
        this->get_parameter("center_map", center_map_);

        // marker_array_ways_pub_(this->create_publisher<visualization_msgs::msg::MarkerArray>("osm_markers", 10));
        marker_array_relations_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("osm_relations", 10);

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
    rclcpp::TimerBase::SharedPtr timer_;
    double center_x_ = 0.0; // Map center X coordinate
    double center_y_ = 0.0; // Map center Y coordinate
    bool data_loaded_ = false; // Flag to indicate if data has been loaded
    double line_width_ = 0.8; // Line width for visualization
    std::string frame_id_ = "map_gyor_0"; // Frame ID for visualization
    std::string filename_; // OSM filename parameter
    bool center_map_ = false; // Center map if true (could be useful for debugging)

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

    void publishMarkers() {
        if (!data_loaded_) {
            return; // Only publish if data is loaded
        }

        // visualization_msgs::msg::MarkerArray marker_array_ways;
        visualization_msgs::msg::MarkerArray marker_array_relations;

        int marker_id = 0;

        // visualize all ways
        // for (const auto& way_pair : ways_) {
        //     const osm::Way* way = way_pair.second;

        //     visualization_msgs::msg::Marker marker;
        //     marker.header.frame_id = "map";
        //     marker.header.stamp = this->now();
        //     marker.ns = "osm_ways";
        //     marker.id = marker_id++;
        //     marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        //     marker.action = visualization_msgs::msg::Marker::ADD;

        //     // Set white color
        //     marker.color.r = 1.0;
        //     marker.color.g = 0.0;
        //     marker.color.b = 0.0;
        //     marker.color.a = 1.0;

        //     // Line width
        //     marker.scale.x = 0.1;

        //     // Add points from way nodes
        //     for (const osm::Node* node : way->nodes()) {
        //         geometry_msgs::msg::Point point;
        //         point.x = node->local_x();
        //         point.y = node->local_y();
        //         point.z = node->ele(); 
        //         marker.points.push_back(point);
        //     }

        //     marker_array_ways.markers.push_back(marker);
        // }

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
        }

        // marker_array_ways_pub_->publish(marker_array_ways);
        marker_array_relations_pub_->publish(marker_array_relations);
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
