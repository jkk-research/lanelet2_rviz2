#include <memory>
#include <locale>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/Geocentric.h>

#include <string>

#include "laneletvisualizer/Lanelet.hpp"
#include "laneletvisualizer/ParkingLot.hpp"
#include "laneletvisualizer/ParkingSpace.hpp"

#include <fstream> // For file I/O


using MarkerArray = visualization_msgs::msg::MarkerArray;

class LaneletVisualizerNode : public rclcpp::Node {
public:
  LaneletVisualizerNode()
    : Node("lanelet_visualizer") {
    this->declare_parameter<std::string>("osm_file_path", "");
    this->declare_parameter<double>("speed_color_max", 90.0);
    this->declare_parameter<std::string>("frame_id", "");
    this->declare_parameter<int>("refresh_freq", 500);

    this->get_parameter("osm_file_path", osm_file_path_);
    this->get_parameter("speed_color_max", speed_color_max_);
    this->get_parameter("frame_id", frame_);
    this->get_parameter("refresh_freq", refresh_freq_);


    std::setlocale(LC_NUMERIC, "C");

    marker_pub_ = this->create_publisher<MarkerArray>("/lanelet_markers", 10);

    // Load the map
    lanelet_map_ = lanelet::load(
      osm_file_path_,
      lanelet::Origin({ 0,0 })
    );

    lanelets_ = &lanelet_map_->laneletLayer;
    auto& nodes_ = lanelet_map_->pointLayer; // the nodes are in the point layer, we need this access to overwrite the internal flawed representation with the local coords

    auto& parking_lots_ = lanelet_map_->polygonLayer;

    auto& lines_ = lanelet_map_->lineStringLayer;

    for (auto& pt : nodes_) { // Iterate over all points in the container

      // Check if the point has the "local_x" attribute
      if (pt.hasAttribute("local_x")) {
        double local_x = pt.attribute("local_x").asDouble().value_or(0.0); // Get the "local_x" attribute
        pt.basicPoint().x() = local_x; // Override the x coordinate

        double local_y = pt.attribute("local_y").asDouble().value_or(0.0); // Get the "local_y" attribute
        pt.basicPoint().y() = local_y; // Override the y coordinate

      }
      else {
        RCLCPP_ERROR(this->get_logger(), "OSM Map is likely unsupported. Contact dev. \n");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
      }
    }


    // fix parking lots
    for (auto& parking_lot : parking_lots_) { // Iterate over all polygons in the container

      // Iterate over all points in the polygon
      for (auto& point : parking_lot) {
        // Check if the point has the "local_x" and "local_y" attributes
        if (point.hasAttribute("local_x") && point.hasAttribute("local_y")) {
          double local_x = point.attribute("local_x").asDouble().value_or(0.0); // Get the "local_x" attribute
          point.basicPoint().x() = local_x; // Override the x coordinate

          double local_y = point.attribute("local_y").asDouble().value_or(0.0); // Get the "local_y" attribute
          point.basicPoint().y() = local_y; // Override the y coordinate
        }
        else {
          RCLCPP_ERROR(this->get_logger(), "OSM Map is likely unsupported. Contact dev. \n");
          rclcpp::shutdown();
          exit(EXIT_FAILURE);
        }
      }
    }

    // fix lines
    for (auto& line : lines_) { // Iterate over all lines in the container
      // Iterate over all points in the line
      for (auto& point : line) {
        // Check if the point has the "local_x" and "local_y" attributes
        if (point.hasAttribute("local_x") && point.hasAttribute("local_y")) {
          double local_x = point.attribute("local_x").asDouble().value_or(0.0); // Get the "local_x" attribute
          point.basicPoint().x() = local_x; // Override the x coordinate

          double local_y = point.attribute("local_y").asDouble().value_or(0.0); // Get the "local_y" attribute
          point.basicPoint().y() = local_y; // Override the y coordinate
        }
        else {
          RCLCPP_ERROR(this->get_logger(), "OSM Map is likely unsupported. Contact dev. \n");
          rclcpp::shutdown();
          exit(EXIT_FAILURE);
        }
      }
    }

    // Timer to periodically publish markers
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000/refresh_freq_),
      std::bind(&LaneletVisualizerNode::publishMarkers, this)
    );
    RCLCPP_INFO(this->get_logger(), "LaneletVisualizerNode initialized");
  }

private:
  void publishMarkers() {
    MarkerArray all_markers;
    int marker_id = 0;
    rclcpp::Time now = this->get_clock()->now();

    for (auto& llt : *lanelets_) {
      laneviz::Lanelet L(llt);
      auto ma = L.draw(frame_, now, marker_id, /*line_width=*/0.05, /*speed_color_max=*/speed_color_max_);
      // append markers
      all_markers.markers.insert(
        all_markers.markers.end(),
        ma.markers.begin(),
        ma.markers.end()
      );
    }

    for (auto& parking_lot : lanelet_map_->polygonLayer) {
      laneviz::ParkingLot PL(parking_lot);
      auto ma = PL.draw(frame_, now, marker_id, /*line_width=*/0.05, /*r=*/0.0f, /*g=*/1.0f, /*b=*/0.0f);
      // append markers
      all_markers.markers.insert(
        all_markers.markers.end(),
        ma.markers.begin(),
        ma.markers.end()
      );
    }

    // draw parkis spaces (only draw if type is "parking_space")
    for (auto& parking_space : lanelet_map_->lineStringLayer) {
      if (parking_space.hasAttribute("type") && parking_space.attribute("type").value() == "parking_space") {
        laneviz::ParkingSpace PS(parking_space);
        auto ma = PS.draw(frame_, now, marker_id, /*line_width=*/0.10);
        // append markers
        all_markers.markers.insert(
          all_markers.markers.end(),
          ma.markers.begin(),
          ma.markers.end()
        );
      }
    }




    marker_pub_->publish(all_markers);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters);

  // Parameters
  std::string osm_file_path_;   // path to file to load
  double speed_color_max_;      // max speed for color coding
  std::string frame_;           // frame to use for visualization
  int refresh_freq_;            // refresh frequency in milliseconds

  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<lanelet::LaneletMap> lanelet_map_;
  const lanelet::LaneletLayer* lanelets_{ nullptr };
};

rcl_interfaces::msg::SetParametersResult LaneletVisualizerNode::parametersCallback(
  const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto& param : parameters) {
    RCLCPP_INFO(this->get_logger(), "Parameter updated: %s = %s",
      param.get_name().c_str(), param.value_to_string().c_str());

    if (param.get_name() == "osm_file_path") {
      osm_file_path_ = param.as_string();
    }

    if (param.get_name() == "speed_color_max") {
      speed_color_max_ = param.as_double();
    }

    if (param.get_name() == "frame") {
      frame_ = param.as_string();
    }

    if (param.get_name() == "refresh_freq") {
      refresh_freq_ = param.as_int();
      // Update the timer with the new frequency
      if (timer_) {
        timer_->cancel();
        timer_ = this->create_wall_timer(
          std::chrono::milliseconds(1000 / refresh_freq_),
          std::bind(&LaneletVisualizerNode::publishMarkers, this)
        );
      }
    }

  }

  return result;
}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaneletVisualizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
