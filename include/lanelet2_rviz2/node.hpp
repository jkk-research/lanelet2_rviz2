#ifndef LANELET2_RVIZ2__NODE_HPP
#define LANELET2_RVIZ2__NODE_HPP

// example node:
// <node id="9209" lat="45.13314197643" lon="16.27647381026">
//   <tag k="local_x" v="372.3642"/>
//   <tag k="local_y" v="-1466.5368"/>
//   <tag k="ele" v="-1.6"/>
// </node>

/// @brief Class to store a single lanelet2 OSM node
class Node {
public:
    Node(const Node&) = default;
    Node(Node&&) = default;
    Node& operator=(const Node&) = default;
    Node& operator=(Node&&) = default;
    ~Node() = default;

    Node(int id, double lat, double lon, double local_x, double local_y, double ele)
        : id_(id), lat_(lat), lon_(lon), local_x_(local_x), local_y_(local_y), ele_(ele) {}

    int id() const { return id_; }
    double lat() const { return lat_; }
    double lon() const { return lon_; }
    double local_x() const { return local_x_; }
    double local_y() const { return local_y_; }
    double ele() const { return ele_; }

private:
    int id_;
    double lat_;
    double lon_;
    double local_x_;
    double local_y_;
    double ele_;
};



#endif  // LANELET2_RVIZ2__NODE_HPP
