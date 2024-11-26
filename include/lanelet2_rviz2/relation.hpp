#ifndef LANELET2_RVIZ2__RELATION_HPP
#define LANELET2_RVIZ2__RELATION_HPP
#include <vector>
#include <string>
#include "way.hpp"

// exmaple relation
//   <relation id="12895">
//     <member type="way" role="left" ref="12888"/>
//     <member type="way" role="right" ref="12887"/>
//     <tag k="type" v="lanelet"/>
//     <tag k="subtype" v="road"/>
//     <tag k="speed_limit" v="50"/>
//     <tag k="location" v="urban"/>
//     <tag k="one_way" v="yes"/>
//   </relation>

namespace osm {

/// @brief Class to store a single lanelet2 OSM relation, which is a part of a street
class Relation {
public:
    Relation(const Relation&) = default;
    Relation(Relation&&) = default;
    Relation& operator=(const Relation&) = default;
    Relation& operator=(Relation&&) = default;
    ~Relation() = default;

    // constructor to fill all fields
    Relation(int id, Way* left, Way* right, std::string type, std::string subtype, int speed_limit, std::string location, bool one_way)
        : id_(id), left_(left), right_(right), type_(type), subtype_(subtype), speed_limit_(speed_limit), location_(location), one_way_(one_way) {}

    int id() const { return id_; }
    Way* left() const { return left_; }
    Way* right() const { return right_; }
    std::string type() const { return type_; }
    std::string subtype() const { return subtype_; }
    int speed_limit() const { return speed_limit_; }
    std::string location() const { return location_; }
    bool one_way() const { return one_way_; }
    
private:
    int id_;
    osm::Way* left_;
    osm::Way* right_;

    std::string type_;
    std::string subtype_;
    int speed_limit_;
    std::string location_;
    bool one_way_;   

};
}; // namespace osm

#endif  // LANELET2_RVIZ2__RELATION_HPP