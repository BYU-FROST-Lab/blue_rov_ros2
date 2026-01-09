import sys

import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def generate_launch_description():
    '''
    Launches the state estimation for the BLUEROV.

    :return: The launch description.
    '''

    sim = "false"  # Default to 'false'
    GPS = "false"  # Default to 'false'
    verbose = "false"  # Default to 'false'
    param_file = '/home/frostlab/config/vehicle_params.yaml'
    namespace = 'coug3'
    with open(param_file, 'r') as f:
        vehicle_params = yaml.safe_load(f)

    if verbose == "true":
        output = 'screen'
    else:
        output = 'log'

    # Get the directory of the launch files
    package_dir = os.path.join(
        get_package_share_directory('cougars_localization'), 'launch')
    imu_package_dir = os.path.join(get_package_share_directory('microstrain_inertial_driver'), 'launch')
    dvl_package_dir = os.path.join(get_package_share_directory('dvl_a50'), 'launch')

    
    launch_actions = []

    launch_actions.extend([
        launch_ros.actions.Node(
            package='dvl_a50', 
            executable='dvl_a50_sensor', 
            name='dvl_a50_node',
            parameters=[param_file],
            namespace=namespace,
        ),
        # launch_ros.actions.Node(
        #     package='seatrac',
        #     executable='modem_pinger',
        #     parameters=[param_file],
        #     namespace=namespace,
        #     output=output,
        # ),
        # launch_ros.actions.Node(
        #     package='mavlink_bridge',
        #     executable='mavlink_bridge',
        #     parameters=[param_file],
        #     namespace=namespace,
        #     output=output,
        # ),
        # # Setup the USBL modem
        # launch_ros.actions.Node(
        #     package='seatrac',
        #     executable='modem',
        #     parameters=[param_file],
        #     namespace=namespace,
        #     output=output,
        # ),
        # launch_ros.actions.Node(
        #     package='cougars_coms',
        #     executable='vehicle_pinger',
        #     parameters=[param_file],
        #     namespace=namespace,
        #     output=output,
        # ),
        
        # launch_ros.actions.Node(
        #     package='cougars_localization',
        #     executable='nmea_constructor.py',
        #     parameters=[param_file],
        #     namespace=namespace,
        #     output=output,
        # ),
        # Pressure sensor for blueROV
        launch_ros.actions.Node(
            package='pressure_sensor',
            executable='pressure_pub',
            name='shallow_pressure_pub',
            parameters=[param_file],
            namespace=namespace,
            output=output,
            remappings=[('pressure/data' , 'pressure/shallow/data')],
        ),

        # Deep Pressure sensor for blueROV
        launch_ros.actions.Node(
            package='pressure_sensor',
            executable='pressure_pub',
            name='deep_pressure_pub',
            parameters=[param_file],
            namespace=namespace,
            output=output,
            remappings=[('pressure/data' , 'pressure/deep/data')],
        ),


    ])

    return launch.LaunchDescription(launch_actions)