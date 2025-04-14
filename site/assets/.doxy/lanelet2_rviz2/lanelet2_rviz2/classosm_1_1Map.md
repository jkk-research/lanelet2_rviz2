

# Class osm::Map



[**ClassList**](annotated.md) **>** [**osm**](namespaceosm.md) **>** [**Map**](classosm_1_1Map.md)


























## Public Attributes

| Type | Name |
| ---: | :--- |
|  int | [**metainfo\_format\_version**](#variable-metainfo_format_version)  <br> |
|  std::string | [**metainfo\_map\_version**](#variable-metainfo_map_version)  <br> |
|  std::vector&lt; [**osm::Node**](classosm_1_1Node.md) \* &gt; | [**nodes**](#variable-nodes)  <br> |
|  std::string | [**osm\_generator**](#variable-osm_generator)  <br> |
|  std::vector&lt; [**osm::Relation**](classosm_1_1Relation.md) \* &gt; | [**relations**](#variable-relations)  <br> |
|  std::vector&lt; [**osm::Way**](classosm_1_1Way.md) \* &gt; | [**ways**](#variable-ways)  <br> |
|  std::string | [**xml\_encoding**](#variable-xml_encoding)  <br> |
|  float | [**xml\_version**](#variable-xml_version)  <br> |
















## Public Functions

| Type | Name |
| ---: | :--- |
|   | [**Map**](#function-map-13) () = default<br> |
|   | [**Map**](#function-map-23) (const [**Map**](classosm_1_1Map.md) &) = default<br> |
|   | [**Map**](#function-map-33) ([**Map**](classosm_1_1Map.md) &&) = default<br> |
|  [**Map**](classosm_1_1Map.md) & | [**operator=**](#function-operator) (const [**Map**](classosm_1_1Map.md) &) = default<br> |
|  [**Map**](classosm_1_1Map.md) & | [**operator=**](#function-operator_1) ([**Map**](classosm_1_1Map.md) &&) = default<br> |
|   | [**~Map**](#function-map) () = default<br> |




























## Public Attributes Documentation




### variable metainfo\_format\_version 

```C++
int osm::Map::metainfo_format_version;
```




<hr>



### variable metainfo\_map\_version 

```C++
std::string osm::Map::metainfo_map_version;
```




<hr>



### variable nodes 

```C++
std::vector<osm::Node*> osm::Map::nodes;
```




<hr>



### variable osm\_generator 

```C++
std::string osm::Map::osm_generator;
```




<hr>



### variable relations 

```C++
std::vector<osm::Relation*> osm::Map::relations;
```




<hr>



### variable ways 

```C++
std::vector<osm::Way*> osm::Map::ways;
```




<hr>



### variable xml\_encoding 

```C++
std::string osm::Map::xml_encoding;
```




<hr>



### variable xml\_version 

```C++
float osm::Map::xml_version;
```




<hr>
## Public Functions Documentation




### function Map [1/3]

```C++
osm::Map::Map () = default
```




<hr>



### function Map [2/3]

```C++
osm::Map::Map (
    const Map &
) = default
```




<hr>



### function Map [3/3]

```C++
osm::Map::Map (
    Map &&
) = default
```




<hr>



### function operator= 

```C++
Map & osm::Map::operator= (
    const Map &
) = default
```




<hr>



### function operator= 

```C++
Map & osm::Map::operator= (
    Map &&
) = default
```




<hr>



### function ~Map 

```C++
osm::Map::~Map () = default
```




<hr>

------------------------------
The documentation for this class was generated from the following file `include/lanelet2_rviz2/map.hpp`

