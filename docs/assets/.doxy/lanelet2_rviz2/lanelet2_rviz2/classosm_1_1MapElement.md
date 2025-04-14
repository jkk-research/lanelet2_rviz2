

# Class osm::MapElement



[**ClassList**](annotated.md) **>** [**osm**](namespaceosm.md) **>** [**MapElement**](classosm_1_1MapElement.md)



_Abstract base class for map elements._ [More...](#detailed-description)

* `#include <mapelement.hpp>`





































## Public Functions

| Type | Name |
| ---: | :--- |
|   | [**MapElement**](#function-mapelement-13) () = default<br> |
|   | [**MapElement**](#function-mapelement-23) (const [**MapElement**](classosm_1_1MapElement.md) &) = default<br> |
|   | [**MapElement**](#function-mapelement-33) ([**MapElement**](classosm_1_1MapElement.md) &&) = default<br> |
| virtual void | [**draw**](#function-draw) () const = 0<br> |
|  [**MapElement**](classosm_1_1MapElement.md) & | [**operator=**](#function-operator) (const [**MapElement**](classosm_1_1MapElement.md) &) = default<br> |
|  [**MapElement**](classosm_1_1MapElement.md) & | [**operator=**](#function-operator_1) ([**MapElement**](classosm_1_1MapElement.md) &&) = default<br> |
| virtual  | [**~MapElement**](#function-mapelement) () = default<br> |




























## Detailed Description


This class serves as a base for all elements in the map that can be drawn, currently most relations, and specific ways. 


    
## Public Functions Documentation




### function MapElement [1/3]

```C++
osm::MapElement::MapElement () = default
```




<hr>



### function MapElement [2/3]

```C++
osm::MapElement::MapElement (
    const MapElement &
) = default
```




<hr>



### function MapElement [3/3]

```C++
osm::MapElement::MapElement (
    MapElement &&
) = default
```




<hr>



### function draw 

```C++
virtual void osm::MapElement::draw () const = 0
```




<hr>



### function operator= 

```C++
MapElement & osm::MapElement::operator= (
    const MapElement &
) = default
```




<hr>



### function operator= 

```C++
MapElement & osm::MapElement::operator= (
    MapElement &&
) = default
```




<hr>



### function ~MapElement 

```C++
virtual osm::MapElement::~MapElement () = default
```




<hr>

------------------------------
The documentation for this class was generated from the following file `include/lanelet2_rviz2/mapelement.hpp`

