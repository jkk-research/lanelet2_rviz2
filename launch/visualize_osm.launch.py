from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='lanelet2_rviz2',
            executable='visualize_osm',
            output='screen',
            parameters=[
                {"frame_id": "map_gyor_0"},
                {"line_width": 0.2},
                {"osm_filename": "/home/zahu/ros2_ws/src/lanelet2_rviz2/src/ZalaZone_Uni_track_full_early.osm"},
                {"center_map": True},
                {"speed_color_max": 90.0}
            ],
        ),
        # Node(
        #     package='pcl_ros',
        #     executable='pcd_to_pointcloud',
        #     name='pcd_to_pointcloud',
        #     parameters=[
        #         {'file_name': '/home/he/dlio_map2.pcd'},
        #         {'tf_frame': 'map_gyor_0'},
        #         {'publishing_period_ms': 1000},
        #     ],
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='gyor0_tf_publisher',
        #     output='screen',
        #     arguments=[
        #         '--x',  '697237.0',
        #         '--y',  '5285644.0',
        #         '--z',  '0.0',
        #         '--qx', '0.0',
        #         '--qy', '0.0',
        #         '--qz', '0.0',
        #         '--qw', '1.0',
        #         '--frame-id',      'map',
        #         '--child-frame-id','map_gyor_0'
        #     ],
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='zala0_tf_publisher',
        #     output='screen',
        #     arguments=[
        #         '--x',  '639770.0',
        #         '--y',  '5195040.0',
        #         '--z',  '0.0',
        #         '--qx', '0.0',
        #         '--qy', '0.0',
        #         '--qz', '0.0',
        #         '--qw', '1.0',
        #         '--frame-id',       'map',
        #         '--child-frame-id', 'map_zala_0'
        #     ],
        # ),
    ])
