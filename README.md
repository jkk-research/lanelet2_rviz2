# OSMVisualizer

**OSMVisualizer** is a ROS 2 node for visualizing OpenStreetMap (OSM) data in RViz2. It dynamically processes `.osm` files and publishes markers for map elements, such as nodes, ways, and relations.

## Features
- **Dynamic File Loading**: Reads the `.osm` filename from a ROS 2 topic (`/osm_filename`).
- **Visualization in RViz2**:
  - Displays map relations as colored line strips.
  - Left ways are shown in **red**.
  - Right ways are shown in **blue**.
- **Reusable Data Structures**: Efficiently processes nodes, ways, and relations while ensuring old data is cleared before new processing.

## Topics

### Published
- `/osm_relations` (visualization_msgs/msg/MarkerArray):
  Marker array for relations (left and right ways).

### Subscribed
- `/osm_filename` (std_msgs/msg/String):
  Receives the `.osm` filename. The file is read and processed dynamically.

## Usage

### 1. Build the Project
Make sure your ROS 2 workspace is set up. Clone this repository into the `src` directory and build it:

```bash
cd ~/ros2_ws
colcon build --packages-select lanelet2_rviz2
source install/setup.bash
```

### 2. Run the Node
Launch the `OSMVisualizer` node:

```bash
ros2 run lanelet2_rviz2 visualize_osm 
```

### 3. Publish a Filename
From another terminal, publish the path to an `.osm` file:

```bash
ros2 topic pub /osm_filename std_msgs/msg/String "{data: '/path/to/your/osm/file.osm'}"
```

The node will process the file, center the map, and start visualizing the data in RViz2.

### 4. Visualize in RViz2
- Open RViz2:
  ```bash
  rviz2
  ```
- Add a `MarkerArray` display.
- Set the topic to `/osm_relations` to visualize relations.

## Acknowledgments
- **[RapidXML](https://rapidxml.sourceforge.net/)** is used for parsing the OSM files. 
