

# File oldrelation.hpp

[**File List**](files.md) **>** [**include**](dir_d44c64559bbebec7f509842c48db8b23.md) **>** [**lanelet2\_rviz2**](dir_65eef65f6947ac43fda5ad768861708a.md) **>** [**oldrelation.hpp**](oldrelation_8hpp.md)

[Go to the documentation of this file](oldrelation_8hpp.md)


```C++
#ifndef LANELET2_RVIZ2__RELATION_HPP
#define LANELET2_RVIZ2__RELATION_HPP
#include <vector>
#include <string>
#include "way.hpp"
using I64 = long long;

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
        I64 ref;
    };

    // constructor to fill all fields
    Relation(I64 id) : id_(id) { }


    I64 id() const { return id_; }
    Way* left() const { return left_; }
    Way* right() const { return right_; }
    std::string type() const { return type_; }
    std::string subtype() const { return subtype_; }
    I64 speed_limit() const { return speed_limit_; }
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
    osm::Way* getMemberByRole(const std::string &role, const std::map<I64, osm::Way*>& ways) const {
        for (const auto &m : members_) {
            if (m.role == role && ways.count(m.ref)) {
                return ways.at(m.ref);
            }
        }
        return nullptr;
    }
    
private:
    I64 id_;
    osm::Way* left_;
    osm::Way* right_;

    std::string type_;
    std::string subtype_;
    I64 speed_limit_;
    std::string location_;
    bool one_way_;   


    std::vector<Member> members_;
    std::map<std::string, std::string> tags_;

};
}; // namespace osm

#endif  // LANELET2_RVIZ2__RELATION_HPP
```


