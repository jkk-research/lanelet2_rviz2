#include "lanelet2_rviz2/OSMVisualizer.hpp"
#include "lanelet2_rviz2/node.hpp"
#include "lanelet2_rviz2/way.hpp"
#include "lanelet2_rviz2/Relation.hpp"


#include "lanelet2_rviz2/rapidxml.hpp"
#include "lanelet2_rviz2/rapidxml_utils.hpp"

#include <cmath>
#include <exception>
#include <stdexcept>
#include <vector>
#include <string>
#include <array>
#include <chrono>

// ROS message includes for header stamp types
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Using the chrono literals is convenient in the implementation.
using namespace std::chrono_literals;

OSMVisualizer::OSMVisualizer()
    : Node("osm_visualizer")
{
    // Declare and get parameters
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
        std::bind(&OSMVisualizer::parametersCallback, this, std::placeholders::_1));

    // Create publisher for marker arrays (relations, speed, one-way arrows)
    marker_array_relations_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("osm_relations", 10);

    // Load and parse the OSM file if provided
    if (!filename_.empty())
    {
        try
        {
            parseOSMFile(filename_.c_str());
            if (center_map_)
            {
                centerCoordinates();
            }
            data_loaded_ = true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse OSM file: %s", e.what());
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "OSM filename parameter is empty.");
    }

    // Create a timer to republish markers every second
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&OSMVisualizer::publishMarkers, this));

    RCLCPP_INFO(this->get_logger(), "OSM Visualizer node started.");
}

void OSMVisualizer::publishMarkers()
{
    if (!data_loaded_)
    {
        RCLCPP_WARN(this->get_logger(), "No data loaded. Skipping marker publication.");
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;


    // virtual visualization_msgs::msg::MarkerArray draw(const std::string &frame_id,
    //                                                 const rclcpp::Time &stamp,
    //                                                 int &marker_id,
    //                                                 double line_width,
    //                                                 double speed_color_max) const = 0;
    // Iterate through relations and add markers to the array
    RCLCPP_INFO(this->get_logger(), "Drawing relations");
    for (const auto &relation_pair : relations_)
    {
        osm::Relation *relation = relation_pair.second;
        if (relation)
        {
            int marker_id = 0;
            auto markers = relation->draw(frame_id_, this->now(), marker_id, line_width_, speed_color_max_);
            marker_array.markers.insert(marker_array.markers.end(), markers.markers.begin(), markers.markers.end());
            RCLCPP_INFO(this->get_logger(), "drawing");
        }
        RCLCPP_INFO(this->get_logger(), "not drawing");

    }

    // Publish the marker array
    RCLCPP_INFO(this->get_logger(), "Publishing marker array with %zu markers", marker_array.markers.size());
    marker_array_relations_pub_->publish(marker_array);
}

rcl_interfaces::msg::SetParametersResult OSMVisualizer::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters)
    {
        RCLCPP_INFO(this->get_logger(), "Parameter updated: %s = %s",
                    param.get_name().c_str(), param.value_to_string().c_str());

        if (param.get_name() == "line_width")
        {
            line_width_ = param.as_double();
        }
        else if (param.get_name() == "frame_id")
        {
            frame_id_ = param.as_string();
        }
        else if (param.get_name() == "osm_filename")
        {
            filename_ = param.as_string();
            try
            {
                parseOSMFile(filename_.c_str());
                if (center_map_)
                {
                    centerCoordinates();
                }
                data_loaded_ = true;
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse OSM file: %s", e.what());
                result.successful = false;
                result.reason = "Failed to parse OSM file";
            }
        }
        else if (param.get_name() == "center_map")
        {
            center_map_ = param.as_bool();
            if (center_map_)
            {
                centerCoordinates();
            }
        }
        else if (param.get_name() == "speed_color_max")
        {
            speed_color_max_ = param.as_double();
        }
    }

    return result;
}



void OSMVisualizer::centerCoordinates()
{
    // Calculate the centroid of the map.
    double sum_x = 0.0;
    double sum_y = 0.0;
    size_t node_count = nodes_.size();

    for (const auto &node_pair : nodes_)
    {
        osm::Node *node = node_pair.second;
        sum_x += node->local_x();
        sum_y += node->local_y();
    }

    if (node_count > 0)
    {
        center_x_ = sum_x / node_count;
        center_y_ = sum_y / node_count;
    }

    // Translate all nodes to center the map.
    for (auto &node_pair : nodes_)
    {
        osm::Node *node = node_pair.second;
        node->set_local_x(node->local_x() - center_x_);
        node->set_local_y(node->local_y() - center_y_);
    }

    RCLCPP_INFO(this->get_logger(), "Map centered at (%.2f, %.2f)", center_x_, center_y_);
}

float OSMVisualizer::mapval(float x, float in_min, float in_max, float out_min, float out_max) const
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void OSMVisualizer::clearData()
{
    for (auto &pair : nodes_)
    {
        delete pair.second;
    }
    nodes_.clear();

    for (auto &pair : ways_)
    {
        delete pair.second;
    }
    ways_.clear();

    for (auto &pair : relations_)
    {
        delete pair.second;
    }
    relations_.clear();
}


void OSMVisualizer::parseOSMFile(const char* filename) {
        // Clear previously loaded data
        clearData();

        rapidxml::file<> xmlFile(filename);
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());

        rapidxml::xml_node<>* root = doc.first_node("osm");

        bool lat_lon_warning_issued = false;

        for (rapidxml::xml_node<>* node = root->first_node("node"); node; node = node->next_sibling("node")) {
            I64 id = std::stoll(node->first_attribute("id")->value());
            double lat = 0, lon = 0, local_x = 0, local_y = 0, ele = 0;

            try {
                if (node->first_attribute("lat") && node->first_attribute("lon")) {
                    lat = std::stod(node->first_attribute("lat")->value());
                    lon = std::stod(node->first_attribute("lon")->value());
                } else {
                    if (!lat_lon_warning_issued) {
                        RCLCPP_WARN(rclcpp::get_logger("node_parser"), "Some nodes have missing latitude or I64itude attributes. Defaulting to 0 for such nodes.");
                        lat_lon_warning_issued = true;
                    }
                }
            } catch (const std::exception& e) {
                if (!lat_lon_warning_issued) {
                    RCLCPP_WARN(rclcpp::get_logger("node_parser"), "Some nodes have malformed latitude or I64itude attributes. Defaulting to 0 for such nodes.");
                    lat_lon_warning_issued = true;
                }
                lat = 0;
                lon = 0;
            }

            // Parse tags: Only convert values for keys that should be numeric.
            for (rapidxml::xml_node<>* tag = node->first_node("tag"); tag; tag = tag->next_sibling("tag")) {
                std::string key = tag->first_attribute("k")->value();
                // Only process numeric tags
                if (key == "local_x" || key == "local_y" || key == "ele") {
                    try {
                        double value = std::stod(tag->first_attribute("v")->value());
                        if (key == "local_x") {
                            local_x = value;
                        } else if (key == "local_y") {
                            local_y = value;
                        } else if (key == "ele") {
                            ele = value;
                        }
                    } catch (const std::exception &e) {
                        RCLCPP_WARN(this->get_logger(), "Failed to parse numeric tag %s with value %s: %s",
                                    key.c_str(), tag->first_attribute("v")->value(), e.what());
                    }
                }
            }


            osm::Node* newNode = new osm::Node(id, lat, lon, local_x, local_y, ele);
            nodes_[id] = newNode;
        }

        // Parse ways
        for (rapidxml::xml_node<>* wayNode = root->first_node("way"); wayNode; wayNode = wayNode->next_sibling("way")) {
            I64 id = std::stoll(wayNode->first_attribute("id")->value());
            osm::Way* way = new osm::Way(id);

            // Parse nd references
            for (rapidxml::xml_node<>* nd = wayNode->first_node("nd"); nd; nd = nd->next_sibling("nd")) {
                I64 ref = std::stoll(nd->first_attribute("ref")->value());
                if (nodes_.count(ref)) {
                    way->add_node(nodes_[ref]);
                }
            }

            bool isline_thin = false;
            
            // Parse and store additional <tag> elements for this way
            for (rapidxml::xml_node<>* tag = wayNode->first_node("tag"); tag; tag = tag->next_sibling("tag")) {
                std::string key = tag->first_attribute("k")->value();
                std::string value = tag->first_attribute("v")->value();
                if (key == "type" && value == "line_thin") {
                    isline_thin = true;
                }
                way->add_tag(key, value);
            }
            

            // if <tag k="type" v="line_thin"/> add it to the list, otherwise don't
            if (isline_thin) {
                ways_[id] = way;
            } // TODO add other types, like parking spaces

        }

        osm::Relation::Member mem;

        // Parse relations
        for (rapidxml::xml_node<>* relationNode = root->first_node("relation"); relationNode; relationNode = relationNode->next_sibling("relation")) {
            I64 id = std::stoll(relationNode->first_attribute("id")->value());
            
            // Create a vector to hold all relation members
            std::vector<osm::Relation::Member> members;
            for (rapidxml::xml_node<>* member = relationNode->first_node("member"); member; member = member->next_sibling("member")) {
                mem.type = member->first_attribute("type")->value();
                mem.role = member->first_attribute("role")->value();
                mem.ref = std::stoll(member->first_attribute("ref")->value());
                members.push_back(mem);
            }

            if (mem.type == "lanelet") {
                // Create the relation object
                osm::LaneletRelation* relation = new osm::LaneletRelation(id);
                relation->set_members(members);

                // Parse and store all tags for this relation
                for (rapidxml::xml_node<>* tag = relationNode->first_node("tag"); tag; tag = tag->next_sibling("tag")) {
                    std::string key = tag->first_attribute("k")->value();
                    std::string value = tag->first_attribute("v")->value();
                    relation->add_tag(key, value);
                }

                relations_[id] = relation; 
            } // TODO gányolás megszűntetése


        }


    }