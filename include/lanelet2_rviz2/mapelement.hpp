#ifndef LANELET2_RVIZ2__MAPELEMENT_HPP
#define LANELET2_RVIZ2__MAPELEMENT_HPP
#include <vector>
#include <string>
#include "node.hpp"

namespace osm {

/**
 * @brief Abstract base class for map elements.
 * @details This class serves as a base for all elements in the map that can be drawn, currently most relations, and specific ways. 
 * 
 */
class MapElement {
public:
    MapElement() = default;
    MapElement(const MapElement&) = default;
    MapElement(MapElement&&) = default;
    MapElement& operator=(const MapElement&) = default;
    MapElement& operator=(MapElement&&) = default;

    virtual ~MapElement() = default;

    virtual void draw() const = 0;

}; 
}; // namespace osm
#endif // LANELET2_RVIZ2__MAPELEMENT_HPP