import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        '/home', 'frostlab',
        'config', 'oculus_driver.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_config,
            description='Path to the ROS2 parameter YAML file'),
        DeclareLaunchArgument(
            'debug',
            default_value='true',
            description='Launch the sonar_image_publisher debug node'),
        DeclareLaunchArgument(
            'namespace',
            default_value='bluerov2',
            description='Namespace for the nodes'),
        Node(
            package='blueprint_oculus_sonar_driver',
            executable='oculus_driver',
            name='oculus_driver',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
            namespace=LaunchConfiguration('namespace'),
        ),
        Node(
            package='blueprint_oculus_sonar_driver',
            executable='sonar_image_publisher.py',
            name='sonar_image_publisher',
            parameters=[LaunchConfiguration('params_file')],
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            condition=IfCondition(LaunchConfiguration('debug')),
        ),
    ])
