

# File map.hpp

[**File List**](files.md) **>** [**include**](dir_d44c64559bbebec7f509842c48db8b23.md) **>** [**lanelet2\_rviz2**](dir_65eef65f6947ac43fda5ad768861708a.md) **>** [**map.hpp**](map_8hpp.md)

[Go to the documentation of this file](map_8hpp.md)


```C++
#ifndef LANELET2_RVIZ2__MAP_HPP
#define LANELET2_RVIZ2__MAP_HPP
#include <vector>
#include <string>
#include "node.hpp"
#include "Relation.hpp"

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

    std::vector<osm::Node*> nodes;
    std::vector<osm::Way*> ways;
    std::vector<osm::Relation*> relations;

}; 
}; // namespace osm
#endif // LANELET2_RVIZ2__MAP_HPP
```


