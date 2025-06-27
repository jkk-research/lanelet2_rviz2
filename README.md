# Building
- `sudo apt install ros-humble-lanelet2`
- clone
- `colcon build --packages-select laneletvisualizer --symlink-install` 
- `source ~/ros2_ws/install/setup.bash`
- `ros2 launch laneletvisualizer visualize_osm.launch.py`