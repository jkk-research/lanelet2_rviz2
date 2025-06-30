# Simultanous visualization of a point cloud and an OSM map in Foxglove / Lichtblick-Suite / RViz2
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from math import pi, degrees


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='lanelet2_rviz2',
            executable='visualize_osm',
            output='screen',
            parameters=[
                {"frame_id": "map_gyor_0"},
                {"line_width": 0.4},
                {"osm_filename": "/home/he/gyor_uni.osm"},
                {"center_map": False},
                {"speed_color_max": 90.0}
            ],
        ),
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'topic', 'pub', '/osm_filename', 
        #         'std_msgs/msg/String', 
        #         "{data: '/home/he/gyor_uni.osm'}"
        #     ],
        #     output='screen'
        # ),
        Node(
            package='pcd_publisher',
            executable='pcd_publisher',
            output='screen',
            parameters=[
                {"pcd_file_path": "/home/he/dlio_map2.pcd"},
                {"frame_id": "map_gyor_x"},
                {"rate": 1},
                {"x_translation": 24.0},
                {"y_translation": 77.0},
                {"z_translation": 0.0},
                {"x_rotation": 0.0},
                {"y_rotation": 0.0},
                {"z_rotation": degrees(-177.1)},
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gyor0_tf_publisher',
            output='screen',
            arguments=[
                '--x',  '697237.0',
                '--y',  '5285644.0',
                '--z',  '0.0',
                '--qx', '0.0',
                '--qy', '0.0',
                '--qz', '0.0',
                '--qw', '1.0',
                '--frame-id',      'map',
                '--child-frame-id','map_gyor_0'
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='zala0_tf_publisher',
            output='screen',
            arguments=[
                '--x',  '639770.0',
                '--y',  '5195040.0',
                '--z',  '0.0',
                '--qx', '0.0',
                '--qy', '0.0',
                '--qz', '0.0',
                '--qw', '1.0',
                '--frame-id',       'map',
                '--child-frame-id', 'map_zala_0'
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gyorx_tf_publisher',
            output='screen',
            arguments=[
                '--x',  '0.0',
                '--y',  '0.0',
                '--z',  '1.0',
                '--qx', '0.0',
                '--qy', '0.0',
                '--qz', '0.0',
                '--qw', '1.0',
                '--frame-id',       'map_gyor_0',
                '--child-frame-id', 'map_gyor_x'
            ],
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            parameters=[
                {'port': 8765},
                {'address': '0.0.0.0'},
                {'tls': False},
                {'certfile': ''},
                {'keyfile': ''},
                #{'topic_whitelist': "'.*'"},
                {'max_qos_depth': 10},
                {'num_threads': 0},
                {'use_sim_time': False},
                {'send_buffer_limit': 100000000 }, # 100MB because of the large point cloud
            ]
        ),
    ])