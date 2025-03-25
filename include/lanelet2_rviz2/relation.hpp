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

    struct Member {
        std::string type;
        std::string role;
        long long ref;
    };

    // constructor to fill all fields
    Relation(long long id) : id_(id) { }


    long long id() const { return id_; }
    Way* left() const { return left_; }
    Way* right() const { return right_; }
    std::string type() const { return type_; }
    std::string subtype() const { return subtype_; }
    long long speed_limit() const { return speed_limit_; }
    std::string location() const { return location_; }
    bool one_way() const { return one_way_; }




    // Set the vector of members parsed from the OSM file
    void set_members(const std::vector<Member>& members) {
        members_ = members;
    }
    
    // (Optional) Getter for the members vector
    const std::vector<Member>& members() const {
        return members_;
    }

    // Add a tag key/value pair to this relation
    void add_tag(const std::string &key, const std::string &value) {
        tags_[key] = value;
    }
    
    // (Optional) Getter for tags
    const std::map<std::string, std::string>& tags() const {
        return tags_;
    }
    
    // (Optional) Helper to get a member by role
    osm::Way* getMemberByRole(const std::string &role, const std::map<long long, osm::Way*>& ways) const {
        for (const auto &m : members_) {
            if (m.role == role && ways.count(m.ref)) {
                return ways.at(m.ref);
            }
        }
        return nullptr;
    }
    
private:
    long long id_;
    osm::Way* left_;
    osm::Way* right_;

    std::string type_;
    std::string subtype_;
    long long speed_limit_;
    std::string location_;
    bool one_way_;   


    std::vector<Member> members_;
    std::map<std::string, std::string> tags_;

};
}; // namespace osm

#endif  // LANELET2_RVIZ2__RELATION_HPP