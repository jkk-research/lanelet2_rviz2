#ifndef LANELET2_RVIZ2__WAY_HPP
#define LANELET2_RVIZ2__WAY_HPP
#include <vector>
#include "node.hpp"

// example way:
//   <way id="11340">
//     <nd ref="9449"/>
//     <nd ref="11344"/>

//     <nd ref="11370"/>
//     <nd ref="11372"/>
//   </way>

namespace osm {

/// @brief Class to store a single lanelet2 OSM way, which is a collection of nodes
class Way {
public:
    Way(const Way&) = default;
    Way(Way&&) = default;
    Way& operator=(const Way&) = default;
    Way& operator=(Way&&) = default;
    ~Way() = default;

    Way(int id) : id_(id) {}

    int id() const { return id_; }

    void add_node(osm::Node* node) { nodes_.push_back(node); }

    const std::vector<osm::Node*>& nodes() const { return nodes_; }

private:
    int id_;
    std::vector<osm::Node*> nodes_;
};

}; // namespace osm

#endif  // LANELET2_RVIZ2__WAY_HPP