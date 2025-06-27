from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='laneletvisualizer',
            executable='laneletviz',
            output='screen',
            parameters=[
                {"frame_id": "map"},
                {"osm_file_path": "/home/zahu/ros2_ws/src/lanelet2_rviz2/src/ZalaZone_SmartCity_2025_03_25_with_parking_lots.osm"},
                {"speed_color_max": 90.0},
                {"refresh_freq": 2},
            ],
        ),
    ])
