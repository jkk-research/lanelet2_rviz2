

# File main.cpp

[**File List**](files.md) **>** [**src**](dir_68267d1309a1af8e8297ef4c3efbcdba.md) **>** [**main.cpp**](main_8cpp.md)

[Go to the documentation of this file](main_8cpp.md)


```C++
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <map>
#include <vector>
#include <numeric>
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include "lanelet2_rviz2/rapidxml.hpp"
#include "lanelet2_rviz2/rapidxml_utils.hpp"
#include "lanelet2_rviz2/node.hpp"
#include "lanelet2_rviz2/way.hpp"
#include "lanelet2_rviz2/Relation.hpp"
#include "lanelet2_rviz2/earcut.hpp"
#include "lanelet2_rviz2/OSMVisualizer.hpp"




int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared < OSMVisualizer > ();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```


