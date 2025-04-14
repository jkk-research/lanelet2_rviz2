

# File way.hpp

[**File List**](files.md) **>** [**include**](dir_d44c64559bbebec7f509842c48db8b23.md) **>** [**lanelet2\_rviz2**](dir_65eef65f6947ac43fda5ad768861708a.md) **>** [**way.hpp**](way_8hpp.md)

[Go to the documentation of this file](way_8hpp.md)


```C++
#ifndef LANELET2_RVIZ2__WAY_HPP
#define LANELET2_RVIZ2__WAY_HPP
#include <vector>
#include "node.hpp"
using I64 = long long;

// example way:
//   <way id="11340">
//     <nd ref="9449"/>
//     <nd ref="11344"/>

//     <nd ref="11370"/>
//     <nd ref="11372"/>
//   </way>

namespace osm {

class Way {
public:
    Way(const Way&) = default;
    Way(Way&&) = default;
    Way& operator=(const Way&) = default;
    Way& operator=(Way&&) = default;
    ~Way() = default;

    Way(I64 id) : id_(id) {}

    I64 id() const { return id_; }

    void add_node(osm::Node* node) { nodes_.push_back(node); }

    const std::vector<osm::Node*>& nodes() const { return nodes_; }

    // Method to add a tag to this way
    void add_tag(const std::string &key, const std::string &value) {
        tags_[key] = value;
    }
    
    // (Optional) Getter to access tags
    const std::map<std::string, std::string>& tags() const {
        return tags_;
    }

private:
    I64 id_;
    std::vector<osm::Node*> nodes_;

    std::map<std::string, std::string> tags_;
};

}; // namespace osm

#endif  // LANELET2_RVIZ2__WAY_HPP
```


