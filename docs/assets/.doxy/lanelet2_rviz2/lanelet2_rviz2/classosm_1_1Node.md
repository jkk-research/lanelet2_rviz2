

# Class osm::Node



[**ClassList**](annotated.md) **>** [**osm**](namespaceosm.md) **>** [**Node**](classosm_1_1Node.md)



_Class to store a single lanelet2 OSM node._ 

* `#include <node.hpp>`





































## Public Functions

| Type | Name |
| ---: | :--- |
|   | [**Node**](#function-node-13) (const [**Node**](classosm_1_1Node.md) &) = default<br> |
|   | [**Node**](#function-node-23) ([**Node**](classosm_1_1Node.md) &&) = default<br> |
|   | [**Node**](#function-node-33) (I64 id, double lat, double lon, double local\_x, double local\_y, double ele) <br> |
|  double | [**ele**](#function-ele) () const<br> |
|  I64 | [**id**](#function-id) () const<br> |
|  double | [**lat**](#function-lat) () const<br> |
|  double | [**local\_x**](#function-local_x) () const<br> |
|  double | [**local\_y**](#function-local_y) () const<br> |
|  double | [**lon**](#function-lon) () const<br> |
|  [**Node**](classosm_1_1Node.md) & | [**operator=**](#function-operator) (const [**Node**](classosm_1_1Node.md) &) = default<br> |
|  [**Node**](classosm_1_1Node.md) & | [**operator=**](#function-operator_1) ([**Node**](classosm_1_1Node.md) &&) = default<br> |
|  void | [**set\_local\_x**](#function-set_local_x) (double local\_x) <br> |
|  void | [**set\_local\_y**](#function-set_local_y) (double local\_y) <br> |
|   | [**~Node**](#function-node) () = default<br> |




























## Public Functions Documentation




### function Node [1/3]

```C++
osm::Node::Node (
    const Node &
) = default
```




<hr>



### function Node [2/3]

```C++
osm::Node::Node (
    Node &&
) = default
```




<hr>



### function Node [3/3]

```C++
inline osm::Node::Node (
    I64 id,
    double lat,
    double lon,
    double local_x,
    double local_y,
    double ele
) 
```




<hr>



### function ele 

```C++
inline double osm::Node::ele () const
```




<hr>



### function id 

```C++
inline I64 osm::Node::id () const
```




<hr>



### function lat 

```C++
inline double osm::Node::lat () const
```




<hr>



### function local\_x 

```C++
inline double osm::Node::local_x () const
```




<hr>



### function local\_y 

```C++
inline double osm::Node::local_y () const
```




<hr>



### function lon 

```C++
inline double osm::Node::lon () const
```




<hr>



### function operator= 

```C++
Node & osm::Node::operator= (
    const Node &
) = default
```




<hr>



### function operator= 

```C++
Node & osm::Node::operator= (
    Node &&
) = default
```




<hr>



### function set\_local\_x 

```C++
inline void osm::Node::set_local_x (
    double local_x
) 
```




<hr>



### function set\_local\_y 

```C++
inline void osm::Node::set_local_y (
    double local_y
) 
```




<hr>



### function ~Node 

```C++
osm::Node::~Node () = default
```




<hr>

------------------------------
The documentation for this class was generated from the following file `include/lanelet2_rviz2/node.hpp`

