

# File node.hpp

[**File List**](files.md) **>** [**include**](dir_d44c64559bbebec7f509842c48db8b23.md) **>** [**lanelet2\_rviz2**](dir_65eef65f6947ac43fda5ad768861708a.md) **>** [**node.hpp**](node_8hpp.md)

[Go to the documentation of this file](node_8hpp.md)


```C++
#ifndef LANELET2_RVIZ2__NODE_HPP
#define LANELET2_RVIZ2__NODE_HPP
using I64 = long long;

// example node:
// <node id="9209" lat="45.13314197643" lon="16.27647381026">
//   <tag k="local_x" v="372.3642"/>
//   <tag k="local_y" v="-1466.5368"/>
//   <tag k="ele" v="-1.6"/>
// </node>

namespace osm {

class Node {
public:
    Node(const Node&) = default;
    Node(Node&&) = default;
    Node& operator=(const Node&) = default;
    Node& operator=(Node&&) = default;
    ~Node() = default;

    Node(I64 id, double lat, double lon, double local_x, double local_y, double ele)
        : id_(id), lat_(lat), lon_(lon), local_x_(local_x), local_y_(local_y), ele_(ele) {}

    I64 id() const { return id_; }
    double lat() const { return lat_; }
    double lon() const { return lon_; }
    double local_x() const { return local_x_; }
    double local_y() const { return local_y_; }
    double ele() const { return ele_; }

    void set_local_x(double local_x) { local_x_ = local_x; }
    void set_local_y(double local_y) { local_y_ = local_y; }

private:
    I64 id_;
    double lat_;
    double lon_;
    double local_x_;
    double local_y_;
    double ele_;
};
}  // namespace osm



#endif  // LANELET2_RVIZ2__NODE_HPP
```


