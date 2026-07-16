from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        '/home/frostlab', 'config', 'stellar_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='stellarhd',
            parameters=[config],
            output='screen',
        ),
    ])