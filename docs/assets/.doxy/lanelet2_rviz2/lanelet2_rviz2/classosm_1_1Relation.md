

# Class osm::Relation



[**ClassList**](annotated.md) **>** [**osm**](namespaceosm.md) **>** [**Relation**](classosm_1_1Relation.md)



_Class to store a single lanelet2 OSM relation, which is a part of a street._ [More...](#detailed-description)

* `#include <oldrelation.hpp>`





Inherited by the following classes: [osm::LaneletRelation](classosm_1_1LaneletRelation.md)










## Classes

| Type | Name |
| ---: | :--- |
| struct | [**Member**](structosm_1_1Relation_1_1Member.md) <br> |






















## Public Functions

| Type | Name |
| ---: | :--- |
|   | [**Relation**](#function-relation-17) (const [**Relation**](classosm_1_1Relation.md) &) = default<br> |
|   | [**Relation**](#function-relation-27) ([**Relation**](classosm_1_1Relation.md) &&) = default<br> |
|   | [**Relation**](#function-relation-37) (I64 id) <br> |
|   | [**Relation**](#function-relation-47) () = default<br> |
|   | [**Relation**](#function-relation-37) (I64 id) <br> |
|   | [**Relation**](#function-relation-17) (const [**Relation**](classosm_1_1Relation.md) &) = default<br> |
|   | [**Relation**](#function-relation-27) ([**Relation**](classosm_1_1Relation.md) &&) = default<br> |
|  void | [**add\_tag**](#function-add_tag-12) (const std::string & key, const std::string & value) <br> |
|  void | [**add\_tag**](#function-add_tag-12) (const std::string & key, const std::string & value) <br> |
| virtual visualization\_msgs::msg::MarkerArray | [**draw**](#function-draw) (const std::string & frame\_id, const rclcpp::Time & stamp, int & marker\_id, double line\_width, double speed\_color\_max) const = 0<br> |
|  [**osm::Way**](classosm_1_1Way.md) \* | [**getMemberByRole**](#function-getmemberbyrole-12) (const std::string & role, const std::map&lt; I64, [**osm::Way**](classosm_1_1Way.md) \* &gt; & ways) const<br> |
|  [**osm::Way**](classosm_1_1Way.md) \* | [**getMemberByRole**](#function-getmemberbyrole-12) (const std::string & role, const std::map&lt; I64, [**osm::Way**](classosm_1_1Way.md) \* &gt; & ways) const<br> |
|  I64 | [**id**](#function-id) () const<br> |
|  [**Way**](classosm_1_1Way.md) \* | [**left**](#function-left) () const<br> |
|  std::string | [**location**](#function-location) () const<br> |
|  const std::vector&lt; [**Member**](structosm_1_1Relation_1_1Member.md) &gt; & | [**members**](#function-members-12) () const<br> |
|  const std::vector&lt; [**Member**](structosm_1_1Relation_1_1Member.md) &gt; & | [**members**](#function-members-12) () const<br> |
|  bool | [**one\_way**](#function-one_way) () const<br> |
|  [**Relation**](classosm_1_1Relation.md) & | [**operator=**](#function-operator) (const [**Relation**](classosm_1_1Relation.md) &) = default<br> |
|  [**Relation**](classosm_1_1Relation.md) & | [**operator=**](#function-operator_1) ([**Relation**](classosm_1_1Relation.md) &&) = default<br> |
|  [**Relation**](classosm_1_1Relation.md) & | [**operator=**](#function-operator) (const [**Relation**](classosm_1_1Relation.md) &) = default<br> |
|  [**Relation**](classosm_1_1Relation.md) & | [**operator=**](#function-operator_1) ([**Relation**](classosm_1_1Relation.md) &&) = default<br> |
|  [**Way**](classosm_1_1Way.md) \* | [**right**](#function-right) () const<br> |
|  void | [**set\_members**](#function-set_members-12) (const std::vector&lt; [**Member**](structosm_1_1Relation_1_1Member.md) &gt; & members) <br> |
|  void | [**set\_members**](#function-set_members-12) (const std::vector&lt; [**Member**](structosm_1_1Relation_1_1Member.md) &gt; & members) <br> |
|  I64 | [**speed\_limit**](#function-speed_limit) () const<br> |
|  std::string | [**subtype**](#function-subtype) () const<br> |
|  const std::map&lt; std::string, std::string &gt; & | [**tags**](#function-tags-12) () const<br> |
|  const std::map&lt; std::string, std::string &gt; & | [**tags**](#function-tags-12) () const<br> |
|  std::string | [**type**](#function-type) () const<br> |
|   | [**~Relation**](#function-relation-12) () = default<br> |
| virtual  | [**~Relation**](#function-relation-22) () = default<br> |




























## Detailed Description


Abstract base class for relations. 


    
## Public Functions Documentation




### function Relation [1/7]

```C++
osm::Relation::Relation (
    const Relation &
) = default
```




<hr>



### function Relation [2/7]

```C++
osm::Relation::Relation (
    Relation &&
) = default
```




<hr>



### function Relation [3/7]

```C++
inline osm::Relation::Relation (
    I64 id
) 
```




<hr>



### function Relation [4/7]

```C++
osm::Relation::Relation () = default
```




<hr>



### function Relation [3/7]

```C++
inline osm::Relation::Relation (
    I64 id
) 
```




<hr>



### function Relation [1/7]

```C++
osm::Relation::Relation (
    const Relation &
) = default
```




<hr>



### function Relation [2/7]

```C++
osm::Relation::Relation (
    Relation &&
) = default
```




<hr>



### function add\_tag [1/2]

```C++
inline void osm::Relation::add_tag (
    const std::string & key,
    const std::string & value
) 
```




<hr>



### function add\_tag [1/2]

```C++
inline void osm::Relation::add_tag (
    const std::string & key,
    const std::string & value
) 
```




<hr>



### function draw 

```C++
virtual visualization_msgs::msg::MarkerArray osm::Relation::draw (
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    int & marker_id,
    double line_width,
    double speed_color_max
) const = 0
```




<hr>



### function getMemberByRole [1/2]

```C++
inline osm::Way * osm::Relation::getMemberByRole (
    const std::string & role,
    const std::map< I64, osm::Way * > & ways
) const
```




<hr>



### function getMemberByRole [1/2]

```C++
inline osm::Way * osm::Relation::getMemberByRole (
    const std::string & role,
    const std::map< I64, osm::Way * > & ways
) const
```




<hr>



### function id 

```C++
inline I64 osm::Relation::id () const
```




<hr>



### function left 

```C++
inline Way * osm::Relation::left () const
```




<hr>



### function location 

```C++
inline std::string osm::Relation::location () const
```




<hr>



### function members [1/2]

```C++
inline const std::vector< Member > & osm::Relation::members () const
```




<hr>



### function members [1/2]

```C++
inline const std::vector< Member > & osm::Relation::members () const
```




<hr>



### function one\_way 

```C++
inline bool osm::Relation::one_way () const
```




<hr>



### function operator= 

```C++
Relation & osm::Relation::operator= (
    const Relation &
) = default
```




<hr>



### function operator= 

```C++
Relation & osm::Relation::operator= (
    Relation &&
) = default
```




<hr>



### function operator= 

```C++
Relation & osm::Relation::operator= (
    const Relation &
) = default
```




<hr>



### function operator= 

```C++
Relation & osm::Relation::operator= (
    Relation &&
) = default
```




<hr>



### function right 

```C++
inline Way * osm::Relation::right () const
```




<hr>



### function set\_members [1/2]

```C++
inline void osm::Relation::set_members (
    const std::vector< Member > & members
) 
```




<hr>



### function set\_members [1/2]

```C++
inline void osm::Relation::set_members (
    const std::vector< Member > & members
) 
```




<hr>



### function speed\_limit 

```C++
inline I64 osm::Relation::speed_limit () const
```




<hr>



### function subtype 

```C++
inline std::string osm::Relation::subtype () const
```




<hr>



### function tags [1/2]

```C++
inline const std::map< std::string, std::string > & osm::Relation::tags () const
```




<hr>



### function tags [1/2]

```C++
inline const std::map< std::string, std::string > & osm::Relation::tags () const
```




<hr>



### function type 

```C++
inline std::string osm::Relation::type () const
```




<hr>



### function ~Relation [1/2]

```C++
osm::Relation::~Relation () = default
```




<hr>



### function ~Relation [2/2]

```C++
virtual osm::Relation::~Relation () = default
```




<hr>

------------------------------
The documentation for this class was generated from the following file `include/lanelet2_rviz2/oldrelation.hpp`

