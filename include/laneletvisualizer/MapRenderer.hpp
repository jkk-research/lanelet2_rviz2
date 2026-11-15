#pragma once

// Turns a Lanelet2 map into a handful of large, opaque TRIANGLE_LIST markers
// (one per namespace / layer) styled like a road map:
//
//   parking        parking lots and parking spaces
//   road           asphalt surface of every drivable lanelet
//   speed_limit    surface tint by speed limit (toggle in RViz)
//   direction      subtle driving-direction chevrons
//   junction_paths faint guides showing how lanes connect inside junctions
//   lane_lines     lane boundaries: solid, or dashed where a lane change is allowed
//   stop_lines     stop lines
//   crosswalk      zebra stripes
//   turn_arrows    pavement arrows in front of junctions
//   traffic_signs  sign symbols
//
// Everything is opaque and layered with small z offsets, because RViz cannot
// depth-sort overlapping transparent markers reliably.

#include <string>
#include <vector>

#include <lanelet2_core/LaneletMap.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "geometry.hpp"

namespace laneviz {

  struct Style {
    double line_width{ 0.15 };       ///< lane boundary width [m]
    double dash_length{ 1.5 };       ///< painted part of a dashed boundary [m]
    double dash_gap{ 1.5 };          ///< gap of a dashed boundary [m]
    double speed_color_max{ 90.0 };  ///< speed limit mapped to the "fast" colour
    bool use_elevation{ true };      ///< false: flatten the map to z = 0

    Color asphalt{ Color::hex(0x343A46) };
    Color marking{ Color::hex(0xE9ECF1) };
    Color marking_yellow{ Color::hex(0xF2C14E) };
    Color chevron{ Color::hex(0x4C5566) };
    Color junction_path{ Color::hex(0x5A6578) };
    Color parking_fill{ Color::hex(0x243447) };
    Color parking_line{ Color::hex(0x6FA8DC) };
    Color sign_stop{ Color::hex(0xE03A45) };
    Color sign_other{ Color::hex(0x3D8BFD) };
    Color speed_slow{ Color::hex(0x7FFFBB) };
    Color speed_mid{ Color::hex(0x0088CC) };
    Color speed_fast{ Color::hex(0xE73666) };
  };

  class MapRenderer {
  public:
    MapRenderer(const lanelet::LaneletMap& map, const Style& style);

    /// The finished marker array; build once, publish as often as needed.
    visualization_msgs::msg::MarkerArray markers(const std::string& frame_id) const;

    size_t vertexCount() const;

  private:
    struct Lane {
      lanelet::Id id{ 0 };
      MeasuredLine left, right, center;
      Polyline outline;  // left + reversed right
      double min_x{ 0 }, min_y{ 0 }, max_x{ 0 }, max_y{ 0 };
      lanelet::Id left_ls{ 0 }, right_ls{ 0 };
      lanelet::Id start_l{ 0 }, start_r{ 0 }, end_l{ 0 }, end_r{ 0 };  // corner point ids
      std::string turn;      // turn_direction tag, may be empty
      double speed{ -1.0 };  // speed limit, < 0 if unknown
      bool junction{ false };
      bool has_arrow{ false };
      std::vector<int> successors;
    };

    struct Boundary {
      MeasuredLine line;
      bool crossable{ false };
      Color color;
      std::vector<int> users;  // lanes bounded by this line
    };

    void collectLanes(const lanelet::LaneletMap& map);
    void connectLanes();
    void detectJunctions();
    void collectBoundaries(const lanelet::LaneletMap& map);

    void drawSurfaces();
    void drawBoundaries();
    void drawJunctionPaths();
    void drawChevrons();
    void drawTurnArrows();
    void drawCrosswalks(const lanelet::LaneletMap& map);
    void drawStopLinesAndSigns(const lanelet::LaneletMap& map);
    void drawParking(const lanelet::LaneletMap& map);

    Polyline toPolyline(const lanelet::ConstLineString3d& ls) const;
    bool onRoad(const Vec3& p) const;
    /// True if p lies well inside a road lanelet other than the ones listed.
    bool coveredByOtherLane(const Vec3& p, const std::vector<int>& own, double margin) const;
    /// 'l', 'r' or 's' for the way a junction lane leaves its predecessor.
    char turnOf(const Lane& lane) const;
    Color speedColor(double speed) const;

    Style style_;
    std::vector<Lane> lanes_;  // drivable lanelets only
    std::vector<Boundary> boundaries_;

    struct Layer {
      const char* ns;
      Mesh mesh;
    };
    Layer parking_{ "parking", {} }, road_{ "road", {} }, speed_{ "speed_limit", {} }, direction_{ "direction", {} },
      junction_paths_{ "junction_paths", {} }, lane_lines_{ "lane_lines", {} }, stop_lines_{ "stop_lines", {} },
      crosswalk_{ "crosswalk", {} }, turn_arrows_{ "turn_arrows", {} }, traffic_signs_{ "traffic_signs", {} };
    std::vector<const Layer*> layers() const {
      return { &parking_, &road_, &speed_, &direction_, &junction_paths_, &lane_lines_, &stop_lines_, &crosswalk_, &turn_arrows_, &traffic_signs_ };
    }
  };

}  // namespace laneviz
