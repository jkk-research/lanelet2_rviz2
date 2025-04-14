

# Class osm::Way



[**ClassList**](annotated.md) **>** [**osm**](namespaceosm.md) **>** [**Way**](classosm_1_1Way.md)



_Class to store a single lanelet2 OSM way, which is a collection of nodes._ 

* `#include <way.hpp>`





































## Public Functions

| Type | Name |
| ---: | :--- |
|   | [**Way**](#function-way-13) (const [**Way**](classosm_1_1Way.md) &) = default<br> |
|   | [**Way**](#function-way-23) ([**Way**](classosm_1_1Way.md) &&) = default<br> |
|   | [**Way**](#function-way-33) (I64 id) <br> |
|  void | [**add\_node**](#function-add_node) ([**osm::Node**](classosm_1_1Node.md) \* node) <br> |
|  void | [**add\_tag**](#function-add_tag) (const std::string & key, const std::string & value) <br> |
|  I64 | [**id**](#function-id) () const<br> |
|  const std::vector&lt; [**osm::Node**](classosm_1_1Node.md) \* &gt; & | [**nodes**](#function-nodes) () const<br> |
|  [**Way**](classosm_1_1Way.md) & | [**operator=**](#function-operator) (const [**Way**](classosm_1_1Way.md) &) = default<br> |
|  [**Way**](classosm_1_1Way.md) & | [**operator=**](#function-operator_1) ([**Way**](classosm_1_1Way.md) &&) = default<br> |
|  const std::map&lt; std::string, std::string &gt; & | [**tags**](#function-tags) () const<br> |
|   | [**~Way**](#function-way) () = default<br> |




























## Public Functions Documentation




### function Way [1/3]

```C++
osm::Way::Way (
    const Way &
) = default
```




<hr>



### function Way [2/3]

```C++
osm::Way::Way (
    Way &&
) = default
```




<hr>



### function Way [3/3]

```C++
inline osm::Way::Way (
    I64 id
) 
```




<hr>



### function add\_node 

```C++
inline void osm::Way::add_node (
    osm::Node * node
) 
```




<hr>



### function add\_tag 

```C++
inline void osm::Way::add_tag (
    const std::string & key,
    const std::string & value
) 
```




<hr>



### function id 

```C++
inline I64 osm::Way::id () const
```




<hr>



### function nodes 

```C++
inline const std::vector< osm::Node * > & osm::Way::nodes () const
```




<hr>



### function operator= 

```C++
Way & osm::Way::operator= (
    const Way &
) = default
```




<hr>



### function operator= 

```C++
Way & osm::Way::operator= (
    Way &&
) = default
```




<hr>



### function tags 

```C++
inline const std::map< std::string, std::string > & osm::Way::tags () const
```




<hr>



### function ~Way 

```C++
osm::Way::~Way () = default
```




<hr>

------------------------------
The documentation for this class was generated from the following file `include/lanelet2_rviz2/way.hpp`

