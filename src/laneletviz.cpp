#include <chrono>
#include <clocale>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>

#include "laneletvisualizer/MapRenderer.hpp"

using MarkerArray = visualization_msgs::msg::MarkerArray;

class LaneletVisualizerNode : public rclcpp::Node {
public:
  LaneletVisualizerNode()
    : Node("lanelet_visualizer") {
    osm_file_path_ = this->declare_parameter<std::string>("osm_file_path", "");
    frame_ = this->declare_parameter<std::string>("frame_id", "map");
    refresh_freq_ = this->declare_parameter<int>("refresh_freq", 1);
    style_.speed_color_max = this->declare_parameter<double>("speed_color_max", style_.speed_color_max);
    style_.line_width = this->declare_parameter<double>("line_width", style_.line_width);
    style_.dash_length = this->declare_parameter<double>("dash_length", style_.dash_length);
    style_.dash_gap = this->declare_parameter<double>("dash_gap", style_.dash_gap);
    style_.use_elevation = this->declare_parameter<bool>("use_elevation", style_.use_elevation);

    // Latched: the map is static, so a display that subscribes with
    // "Transient Local" durability gets it without any republishing.
    marker_pub_ = this->create_publisher<MarkerArray>("/lanelet_markers", rclcpp::QoS(1).transient_local());

    if (!loadMap()) {
      throw std::runtime_error("could not load a usable map from '" + osm_file_path_ + "'");
    }
    rebuild();
    resetTimer();

    param_cb_ = this->add_on_set_parameters_callback(
      std::bind(&LaneletVisualizerNode::parametersCallback, this, std::placeholders::_1));
  }

private:
  bool loadMap() {
    std::setlocale(LC_NUMERIC, "C");  // the OSM parser must read "1.5", whatever the system locale
    lanelet::ErrorMessages errors;
    std::unique_ptr<lanelet::LaneletMap> map;
    try {
      map = lanelet::load(osm_file_path_, lanelet::Origin({ 0, 0 }), &errors);
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load '%s': %s", osm_file_path_.c_str(), e.what());
      return false;
    }
    for (const auto& e : errors) RCLCPP_WARN(this->get_logger(), "Map loader: %s", e.c_str());

    // The maps we use carry metric coordinates in local_x / local_y tags; take
    // those instead of the projected lat / lon. Linestrings, polygons and
    // lanelets share these points, so fixing the point layer fixes everything.
    size_t missing = 0;
    for (auto& pt : map->pointLayer) {
      if (pt.hasAttribute("local_x") && pt.hasAttribute("local_y")) {
        pt.x() = pt.attribute("local_x").asDouble().value_or(0.0);
        pt.y() = pt.attribute("local_y").asDouble().value_or(0.0);
      }
      else {
        ++missing;
      }
    }
    if (missing > 0) {
      RCLCPP_ERROR(this->get_logger(),
        "Unsupported map: %zu of %zu nodes have no local_x / local_y tags.", missing, map->pointLayer.size());
      return false;
    }
    lanelet_map_ = std::move(map);
    return true;
  }

  void rebuild() {
    const auto t0 = std::chrono::steady_clock::now();
    const laneviz::MapRenderer renderer(*lanelet_map_, style_);
    markers_ = renderer.markers(frame_);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    RCLCPP_INFO(this->get_logger(), "Map with %zu lanelets rendered into %zu markers (%zu triangles) in %.0f ms",
      lanelet_map_->laneletLayer.size(), markers_.markers.size(), renderer.vertexCount() / 3, ms);
    marker_pub_->publish(markers_);
  }

  void resetTimer() {
    if (timer_) timer_->cancel();
    timer_.reset();
    if (refresh_freq_ > 0) {
      // Only for displays that subscribe late with volatile durability; the
      // message is cached, nothing is recomputed.
      timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / refresh_freq_),
        [this]() { marker_pub_->publish(markers_); });
    }
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Validate everything first, so a rejected set leaves the node untouched.
    for (const auto& param : parameters) {
      const std::string& name = param.get_name();
      if ((name == "line_width" || name == "dash_length" || name == "dash_gap" || name == "speed_color_max") && param.as_double() <= 0.0) {
        result.successful = false;
        result.reason = name + " must be positive";
        return result;
      }
      if (name == "refresh_freq" && param.as_int() < 0) {
        result.successful = false;
        result.reason = "refresh_freq must be >= 0 (0 publishes once, latched)";
        return result;
      }
    }

    bool reload = false, redraw = false, retime = false;
    const std::string previous_path = osm_file_path_;
    for (const auto& param : parameters) {
      const std::string& name = param.get_name();
      if (name == "osm_file_path") { osm_file_path_ = param.as_string(); reload = true; }
      else if (name == "frame_id") { frame_ = param.as_string(); redraw = true; }
      else if (name == "speed_color_max") { style_.speed_color_max = param.as_double(); redraw = true; }
      else if (name == "line_width") { style_.line_width = param.as_double(); redraw = true; }
      else if (name == "dash_length") { style_.dash_length = param.as_double(); redraw = true; }
      else if (name == "dash_gap") { style_.dash_gap = param.as_double(); redraw = true; }
      else if (name == "use_elevation") { style_.use_elevation = param.as_bool(); redraw = true; }
      else if (name == "refresh_freq") { refresh_freq_ = static_cast<int>(param.as_int()); retime = true; }
    }

    if (reload && !loadMap()) {  // the previous map stays on screen
      osm_file_path_ = previous_path;
      result.successful = false;
      result.reason = "could not load the map";
      return result;
    }
    if (reload || redraw) rebuild();
    if (retime) resetTimer();
    return result;
  }

  std::string osm_file_path_;  // map file to load
  std::string frame_;          // frame the map is drawn in
  int refresh_freq_;           // republish rate [Hz], 0 = publish once (latched)
  laneviz::Style style_;

  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  std::unique_ptr<lanelet::LaneletMap> lanelet_map_;
  MarkerArray markers_;  // built once per map / style change
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LaneletVisualizerNode>());
  }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("lanelet_visualizer"), "%s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
