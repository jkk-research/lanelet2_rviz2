

# File triangulate\_polygon.hpp



[**FileList**](files.md) **>** [**include**](dir_d44c64559bbebec7f509842c48db8b23.md) **>** [**lanelet2\_rviz2**](dir_65eef65f6947ac43fda5ad768861708a.md) **>** [**triangulate\_polygon.hpp**](triangulate__polygon_8hpp.md)

[Go to the source code of this file](triangulate__polygon_8hpp_source.md)



* `#include <geometry_msgs/msg/point32.hpp>`
* `#include <visualization_msgs/msg/marker_array.hpp>`
* `#include <rclcpp/rclcpp.hpp>`
* `#include "lanelet2_rviz2/earcut.hpp"`





































## Public Functions

| Type | Name |
| ---: | :--- |
|  visualization\_msgs::msg::Marker | [**triangulatePolygon**](#function-triangulatepolygon) (const geometry\_msgs::msg::PolygonStamped & polygon, double r, double g, double b, double a, I64 id) <br>_Triangulates a polygon to produce a Marker message._  |




























## Public Functions Documentation




### function triangulatePolygon 

_Triangulates a polygon to produce a Marker message._ 
```C++
inline visualization_msgs::msg::Marker triangulatePolygon (
    const geometry_msgs::msg::PolygonStamped & polygon,
    double r,
    double g,
    double b,
    double a,
    I64 id
) 
```



This is a stub for your own triangulation routine.




**Parameters:**


* `polygon` The input polygon. 
* `r` Color red component. 
* `g` Color green component. 
* `b` Color blue component. 
* `a` Color alpha component. 
* `id` Unique marker id. 



**Returns:**

A Marker message representing the triangulated polygon. 





        

<hr>

------------------------------
The documentation for this class was generated from the following file `include/lanelet2_rviz2/triangulate_polygon.hpp`

