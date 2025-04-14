

# File mapelement.hpp

[**File List**](files.md) **>** [**include**](dir_d44c64559bbebec7f509842c48db8b23.md) **>** [**lanelet2\_rviz2**](dir_65eef65f6947ac43fda5ad768861708a.md) **>** [**mapelement.hpp**](mapelement_8hpp.md)

[Go to the documentation of this file](mapelement_8hpp.md)


```C++
#ifndef LANELET2_RVIZ2__MAPELEMENT_HPP
#define LANELET2_RVIZ2__MAPELEMENT_HPP
#include <vector>
#include <string>
#include "node.hpp"

namespace osm {

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
```


