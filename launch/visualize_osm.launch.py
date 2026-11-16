import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_rviz_config = os.path.join(
        get_package_share_directory('lanelet2_rviz2'), 'etc', 'zalazone.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'osm_file_path',
            # default_value='/mnt/c/bag/zala/ZalaZone_SmartCityZone_lanelet2_map_2025_05_21.osm',
            # default_value='/mnt/c/bag/jkk_dataset_04_campus_2026_07_20/lanelet2_map_2026_07_20.osm', # download from https://zenodo.org/records/21871328
            default_value='/mnt/c/bag/zala/ZalaZone_SmartCityZone_lanelet2_map_2025_05_21.osm', # download from https://github.com/jkk-research/lanelet2_maps
            description='Lanelet2 .osm map to visualize'),
        DeclareLaunchArgument(
            'frame_id', default_value='map',
            description='Frame the map is drawn in'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2 together with the visualizer'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz_config,
            description='RViz2 configuration file'),

        Node(
            package='lanelet2_rviz2',
            executable='lanelet2_rviz2',
            output='screen',
            parameters=[
                {'frame_id': LaunchConfiguration('frame_id')},
                {'osm_file_path': ParameterValue(LaunchConfiguration('osm_file_path'), value_type=str)},
                {'speed_color_max': 90.0},
                {'refresh_freq': 1},
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
