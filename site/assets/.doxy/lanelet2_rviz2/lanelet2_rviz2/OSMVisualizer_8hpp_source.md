

# File OSMVisualizer.hpp

[**File List**](files.md) **>** [**include**](dir_d44c64559bbebec7f509842c48db8b23.md) **>** [**lanelet2\_rviz2**](dir_65eef65f6947ac43fda5ad768861708a.md) **>** [**OSMVisualizer.hpp**](OSMVisualizer_8hpp.md)

[Go to the documentation of this file](OSMVisualizer_8hpp.md)


```C++
#ifndef LANELET2_RVIZ2__OSM_VISUALIZER_HPP
#define LANELET2_RVIZ2__OSM_VISUALIZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <map>
#include <vector>
#include <string>
#include <array>
#include <stdexcept>

// ROS message includes
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "lanelet2_rviz2/node.hpp"
#include "lanelet2_rviz2/way.hpp"
#include "lanelet2_rviz2/Relation.hpp"

// Include Earcut (used for triangulation)
#include "earcut.hpp"

// Forward declarations for types provided by the OSM parser

using I64 = long long;

class OSMVisualizer : public rclcpp::Node {
public:
  OSMVisualizer();
  // Optionally you could implement a destructor to clear data:
  // ~OSMVisualizer();

private:
  // Callback for parameter changes
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters);



  // Computes the center of map coordinates and recenters the nodes.
  void centerCoordinates();

  // Helper: maps a value from one range to another.
  float mapval(float x, float in_min, float in_max, float out_min, float out_max) const;

  // Publishes markers (lines, polygons, and arrows) for visualizing OSM data.
  void publishMarkers();

  // Clears any stored data (nodes, ways, relations)
  void clearData();

  // Parses the OSM file; should populate nodes_, ways_, and relations_.
  // (Note: The implementation is not provided in the original file.)
  void parseOSMFile(const char* filename);

private:
  std::map<I64, osm::Node*> nodes_;
  std::map<I64, osm::Way*> ways_;
  std::map<I64, osm::Relation*> relations_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_relations_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  double center_x_ = 0.0;
  double center_y_ = 0.0;
  bool data_loaded_ = false;
  double line_width_ = 0.8;
  std::string frame_id_ = "map_gyor_0";
  std::string filename_;      // OSM filename parameter
  bool center_map_ = false;   // Whether to center the map
  double speed_color_max_ = 90.0;
};

#endif  // LANELET2_RVIZ2__OSM_VISUALIZER_HPP
```


