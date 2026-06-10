from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path


def generate_launch_description():
    home_path = os.getenv('HOME', '/home/frostlab')
    config_path = os.path.join(home_path, 'config')
    # Mapviz origins param file
    mapviz_origins_path = os.path.join(
        config_path, 'mapviz_origins.yaml'
    )
    base_station_params = os.path.join(config_path, 'base_station_params.yaml')
    rqt_perspective_file = os.path.join(config_path, "bluerov2.perspective")
    
    mapviz_origins = Path(mapviz_origins_path).read_text()
  
    return LaunchDescription([
        # Set environment variable
        # Declare launch arguments
        DeclareLaunchArgument('origin', default_value='bluerov_site'),

        # Node for mapviz
        Node(
            package='mapviz',
            executable='mapviz',
            name='mapviz',
            parameters=[base_station_params],
        ),

        # Node for initialize_origin
        Node(
            package='swri_transform_util',
            executable='initialize_origin.py',
            name='initialize_origin',
            remappings=[('/fix', '/bluerov2/imu/nav_sat_fix')],
            parameters=[
                {'local_xy_origin': LaunchConfiguration('origin')},
                {'local_xy_origins': mapviz_origins},
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='swri_transform',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'origin'],
            parameters=[base_station_params],
        ),
        Node(
            package='blueprint_oculus_sonar_driver',
            executable='sonar_image_publisher.py',
            name='sonar_image_publisher',
            parameters=[base_station_params],
            namespace='bluerov2',
        ),
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui',
            arguments=["--perspective-file", rqt_perspective_file],
            namespace='bluerov2',
            parameters=[base_station_params],
        ),

    ])
