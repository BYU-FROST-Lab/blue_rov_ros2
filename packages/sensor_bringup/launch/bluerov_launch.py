import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    '''
    Launches the state estimation for the BLUEROV.

    :return: The launch description.
    '''
    sbg_config = os.path.join(
        '/home',
        'frostlab',
        'config',
        'sbg_driver_params.yaml'
    )
    ntrip_config = os.path.join(
        '/home',
        'frostlab',
        'config',
        'ntrip_client_params.yaml'
    )
    nmea_config = os.path.join(
        '/home',
        'frostlab',
        'config',
        'nmea_gpsd_params.yaml'
    )

    sim = "false"  # Default to 'false'
    GPS = "false"  # Default to 'false'
    verbose = "false"  # Default to 'false'
    param_file = '/home/frostlab/config/vehicle_params.yaml'

    if verbose == "true":
        output = 'screen'
    else:
        output = 'log'

    launch_actions = []

    launch_actions.extend([
        DeclareLaunchArgument('namespace', default_value='bluerov2'),
        launch_ros.actions.Node(
            package='dvl_a50', 
            executable='dvl_a50_sensor', 
            name='dvl_a50_node',
            parameters=[param_file],
            namespace=LaunchConfiguration('namespace'),
        ),
        launch_ros.actions.Node(
            package='sensor_bringup', 
            executable='dvl_converter', 
            name='dvl_to_twist_node',
            parameters=[param_file],
            namespace=LaunchConfiguration('namespace'),
        ),

        # launch_ros.actions.Node(
        #     package='seatrac',
        #     executable='modem_pinger',
        #     parameters=[param_file],
        #     namespace=LaunchConfiguration('namespace'),
        #     output=output,
        # ),
        launch_ros.actions.Node(
            package='mavlink_bridge',
            executable='mavlink_bridge',
            parameters=[param_file],
            namespace=LaunchConfiguration('namespace'),
            output=output,
        ),
        # # Setup the USBL modem
        # launch_ros.actions.Node(
        #     package='seatrac',
        #     executable='modem',
        #     parameters=[param_file],
        #     namespace=LaunchConfiguration('namespace'),
        #     output=output,
        # ),
        # launch_ros.actions.Node(
        #     package='cougars_coms',
        #     executable='vehicle_pinger',
        #     parameters=[param_file],
        #     namespace=LaunchConfiguration('namespace'),
        #     output=output,
        # ),
        
        # launch_ros.actions.Node(
        #     package='cougars_localization',
        #     executable='nmea_constructor.py',
        #     parameters=[param_file],
        #     namespace=LaunchConfiguration('namespace'),
        #     output=output,LaunchConfiguration
        # ),


        ################ Pressure sensor #########
        launch_ros.actions.Node(
            package='pressure_sensor',
            executable='pressure_pub',
            name='pressure_pub',
            parameters=[param_file],
            namespace=[LaunchConfiguration('namespace'), '/shallow'],
            output=output,
        ),
        launch_ros.actions.Node(
            package='pressure_sensor', 
            executable='pressure_to_depth', 
            name='depth_converter',
            parameters=[param_file],
            namespace=[LaunchConfiguration('namespace'), '/shallow'],
        ),
        # Deep Pressure sensor for blueROV
        launch_ros.actions.Node(
            package='pressure_sensor',
            executable='pressure_pub',
            name='pressure_pub',
            parameters=[param_file],
            namespace=[LaunchConfiguration('namespace'), '/deep'],
            output=output,
        ),
        launch_ros.actions.Node(
            package='pressure_sensor', 
            executable='pressure_to_depth', 
            name='depth_converter',
            parameters=[param_file],
            namespace=[LaunchConfiguration('namespace'), '/deep'],
        ),

        launch_ros.actions.Node(
            package='sbg_driver',
        #	name='sbg_device_1',
            executable = 'sbg_device',
            output = 'screen',
            namespace=LaunchConfiguration('namespace'),
            parameters = [sbg_config]
        ),
        launch_ros.actions.Node(
            package='nmea_gpsd',
            executable='nmea_gpsd_udp',
            output='screen',
            namespace=LaunchConfiguration('namespace'),
            parameters=[nmea_config]
        ),
        launch_ros.actions.Node(
            package='ntrip_client',
            executable='ntrip_ros.py',
            name='ntrip_client',
            namespace=LaunchConfiguration('namespace'),
            parameters=[ntrip_config],
        ),


    ])

    return launch.LaunchDescription(launch_actions)