

# Class osm::LaneletRelation



[**ClassList**](annotated.md) **>** [**osm**](namespaceosm.md) **>** [**LaneletRelation**](classosm_1_1LaneletRelation.md)



_Class representing a lanelet relation._ [More...](#detailed-description)

* `#include <Relation.hpp>`



Inherits the following classes: [osm::Relation](classosm_1_1Relation.md)






















































## Public Functions

| Type | Name |
| ---: | :--- |
|   | [**LaneletRelation**](#function-laneletrelation-13) (I64 id) <br> |
|   | [**LaneletRelation**](#function-laneletrelation-23) (const [**LaneletRelation**](classosm_1_1LaneletRelation.md) &) = default<br> |
|   | [**LaneletRelation**](#function-laneletrelation-33) ([**LaneletRelation**](classosm_1_1LaneletRelation.md) &&) = default<br> |
| virtual visualization\_msgs::msg::MarkerArray | [**draw**](#function-draw) (const std::string & frame\_id, const rclcpp::Time & stamp, int & marker\_id, double line\_width, double speed\_color\_max) override const<br>_Create visualization markers for this lanelet relation._  |
|  [**Way**](classosm_1_1Way.md) \* | [**left**](#function-left) () const<br> |
|  std::string | [**location**](#function-location) () const<br> |
|  bool | [**one\_way**](#function-one_way) () const<br> |
|  [**LaneletRelation**](classosm_1_1LaneletRelation.md) & | [**operator=**](#function-operator) (const [**LaneletRelation**](classosm_1_1LaneletRelation.md) &) = default<br> |
|  [**LaneletRelation**](classosm_1_1LaneletRelation.md) & | [**operator=**](#function-operator_1) ([**LaneletRelation**](classosm_1_1LaneletRelation.md) &&) = default<br> |
|  [**Way**](classosm_1_1Way.md) \* | [**right**](#function-right) () const<br> |
|  I64 | [**speed\_limit**](#function-speed_limit) () const<br> |
|  std::string | [**turn\_direction**](#function-turn_direction) () const<br> |
| virtual  | [**~LaneletRelation**](#function-laneletrelation) () = default<br> |


## Public Functions inherited from osm::Relation

See [osm::Relation](classosm_1_1Relation.md)

| Type | Name |
| ---: | :--- |
|   | [**Relation**](classosm_1_1Relation.md#function-relation-17) (const [**Relation**](classosm_1_1Relation.md) &) = default<br> |
|   | [**Relation**](classosm_1_1Relation.md#function-relation-27) ([**Relation**](classosm_1_1Relation.md) &&) = default<br> |
|   | [**Relation**](classosm_1_1Relation.md#function-relation-37) (I64 id) <br> |
|   | [**Relation**](classosm_1_1Relation.md#function-relation-47) () = default<br> |
|   | [**Relation**](classosm_1_1Relation.md#function-relation-37) (I64 id) <br> |
|   | [**Relation**](classosm_1_1Relation.md#function-relation-17) (const [**Relation**](classosm_1_1Relation.md) &) = default<br> |
|   | [**Relation**](classosm_1_1Relation.md#function-relation-27) ([**Relation**](classosm_1_1Relation.md) &&) = default<br> |
|  void | [**add\_tag**](classosm_1_1Relation.md#function-add_tag-12) (const std::string & key, const std::string & value) <br> |
|  void | [**add\_tag**](classosm_1_1Relation.md#function-add_tag-12) (const std::string & key, const std::string & value) <br> |
| virtual visualization\_msgs::msg::MarkerArray | [**draw**](classosm_1_1Relation.md#function-draw) (const std::string & frame\_id, const rclcpp::Time & stamp, int & marker\_id, double line\_width, double speed\_color\_max) const = 0<br> |
|  [**osm::Way**](classosm_1_1Way.md) \* | [**getMemberByRole**](classosm_1_1Relation.md#function-getmemberbyrole-12) (const std::string & role, const std::map&lt; I64, [**osm::Way**](classosm_1_1Way.md) \* &gt; & ways) const<br> |
|  [**osm::Way**](classosm_1_1Way.md) \* | [**getMemberByRole**](classosm_1_1Relation.md#function-getmemberbyrole-12) (const std::string & role, const std::map&lt; I64, [**osm::Way**](classosm_1_1Way.md) \* &gt; & ways) const<br> |
|  I64 | [**id**](classosm_1_1Relation.md#function-id) () const<br> |
|  [**Way**](classosm_1_1Way.md) \* | [**left**](classosm_1_1Relation.md#function-left) () const<br> |
|  std::string | [**location**](classosm_1_1Relation.md#function-location) () const<br> |
|  const std::vector&lt; [**Member**](structosm_1_1Relation_1_1Member.md) &gt; & | [**members**](classosm_1_1Relation.md#function-members-12) () const<br> |
|  const std::vector&lt; [**Member**](structosm_1_1Relation_1_1Member.md) &gt; & | [**members**](classosm_1_1Relation.md#function-members-12) () const<br> |
|  bool | [**one\_way**](classosm_1_1Relation.md#function-one_way) () const<br> |
|  [**Relation**](classosm_1_1Relation.md) & | [**operator=**](classosm_1_1Relation.md#function-operator) (const [**Relation**](classosm_1_1Relation.md) &) = default<br> |
|  [**Relation**](classosm_1_1Relation.md) & | [**operator=**](classosm_1_1Relation.md#function-operator_1) ([**Relation**](classosm_1_1Relation.md) &&) = default<br> |
|  [**Relation**](classosm_1_1Relation.md) & | [**operator=**](classosm_1_1Relation.md#function-operator) (const [**Relation**](classosm_1_1Relation.md) &) = default<br> |
|  [**Relation**](classosm_1_1Relation.md) & | [**operator=**](classosm_1_1Relation.md#function-operator_1) ([**Relation**](classosm_1_1Relation.md) &&) = default<br> |
|  [**Way**](classosm_1_1Way.md) \* | [**right**](classosm_1_1Relation.md#function-right) () const<br> |
|  void | [**set\_members**](classosm_1_1Relation.md#function-set_members-12) (const std::vector&lt; [**Member**](structosm_1_1Relation_1_1Member.md) &gt; & members) <br> |
|  void | [**set\_members**](classosm_1_1Relation.md#function-set_members-12) (const std::vector&lt; [**Member**](structosm_1_1Relation_1_1Member.md) &gt; & members) <br> |
|  I64 | [**speed\_limit**](classosm_1_1Relation.md#function-speed_limit) () const<br> |
|  std::string | [**subtype**](classosm_1_1Relation.md#function-subtype) () const<br> |
|  const std::map&lt; std::string, std::string &gt; & | [**tags**](classosm_1_1Relation.md#function-tags-12) () const<br> |
|  const std::map&lt; std::string, std::string &gt; & | [**tags**](classosm_1_1Relation.md#function-tags-12) () const<br> |
|  std::string | [**type**](classosm_1_1Relation.md#function-type) () const<br> |
|   | [**~Relation**](classosm_1_1Relation.md#function-relation-12) () = default<br> |
| virtual  | [**~Relation**](classosm_1_1Relation.md#function-relation-22) () = default<br> |






















































## Detailed Description


This class not only contains lanelet–specific attributes but now also implements drawing of its own visualization markers. 


    
## Public Functions Documentation




### function LaneletRelation [1/3]

```C++
inline osm::LaneletRelation::LaneletRelation (
    I64 id
) 
```




<hr>



### function LaneletRelation [2/3]

```C++
osm::LaneletRelation::LaneletRelation (
    const LaneletRelation &
) = default
```




<hr>



### function LaneletRelation [3/3]

```C++
osm::LaneletRelation::LaneletRelation (
    LaneletRelation &&
) = default
```




<hr>



### function draw 

_Create visualization markers for this lanelet relation._ 
```C++
inline virtual visualization_msgs::msg::MarkerArray osm::LaneletRelation::draw (
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    int & marker_id,
    double line_width,
    double speed_color_max
) override const
```





**Parameters:**


* `frame_id` The coordinate frame. 
* `stamp` The current time stamp. 
* `marker_id` A counter for unique marker IDs (which will be incremented). 
* `line_width` The desired line width for drawing. 
* `speed_color_max` The maximum speed used for color mapping. 



**Returns:**

MarkerArray message with all markers for the relation. 





        
Implements [*osm::Relation::draw*](classosm_1_1Relation.md#function-draw)


<hr>



### function left 

```C++
inline Way * osm::LaneletRelation::left () const
```




<hr>



### function location 

```C++
inline std::string osm::LaneletRelation::location () const
```




<hr>



### function one\_way 

```C++
inline bool osm::LaneletRelation::one_way () const
```




<hr>



### function operator= 

```C++
LaneletRelation & osm::LaneletRelation::operator= (
    const LaneletRelation &
) = default
```




<hr>



### function operator= 

```C++
LaneletRelation & osm::LaneletRelation::operator= (
    LaneletRelation &&
) = default
```




<hr>



### function right 

```C++
inline Way * osm::LaneletRelation::right () const
```




<hr>



### function speed\_limit 

```C++
inline I64 osm::LaneletRelation::speed_limit () const
```




<hr>



### function turn\_direction 

```C++
inline std::string osm::LaneletRelation::turn_direction () const
```




<hr>



### function ~LaneletRelation 

```C++
virtual osm::LaneletRelation::~LaneletRelation () = default
```




<hr>

------------------------------
The documentation for this class was generated from the following file `include/lanelet2_rviz2/Relation.hpp`

