#ifndef LANELET2_RVIZ2__MAP_HPP
#define LANELET2_RVIZ2__MAP_HPP
#include <vector>
#include <string>
#include "node.hpp"
#include "relation.hpp"

namespace osm {

class Map {
public:
    Map() = default;
    Map(const Map&) = default;
    Map(Map&&) = default;
    Map& operator=(const Map&) = default;
    Map& operator=(Map&&) = default;
    ~Map() = default;

    float xml_version;
    std::string xml_encoding;
    std::string osm_generator;

    int metainfo_format_version;
    std::string metainfo_map_version;

    std::vector<osm::Relation*> relations;

}; 
}; // namespace osm
#endif // LANELET2_RVIZ2__MAP_HPP