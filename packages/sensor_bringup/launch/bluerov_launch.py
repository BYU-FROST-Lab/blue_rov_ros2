import launch
import launch_ros.actions
from launch_ros.actions import Node, PushRosNamespace
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os
import yaml
from launch.substitutions import Command

def generate_launch_description():
    '''
    Launches the state estimation for the BLUEROV.

    :return: The launch description.
    '''
    vehicle_params_path = '/home/frostlab/config/vehicle_params.yaml'
    with open(vehicle_params_path) as f:
        _vp = yaml.safe_load(f)
    _origin = _vp['/**']['ros__parameters']['origin']
    sbg_datum = {
        'odometry.datumLat': float(_origin['latitude']),
        'odometry.datumLon': float(_origin['longitude']),
        'odometry.datumAlt': float(_origin['altitude']),
    }

    urdf_path = os.path.join(
        get_package_share_directory('sensor_bringup'),
        'urdf',
        'bluerov2.urdf.xacro'
    )
    robot_description = Command(['xacro ', urdf_path])

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
    dvl_a50_config = os.path.join(
        '/home',
        'frostlab',
        'config',
        'dvl_params.yaml'
    )
    time_sync_config = os.path.join(
        '/home',
        'frostlab',
        'config',
        'time_sync_utils.yaml'
    )
    diagnostic_agg_config = os.path.join(
        '/home',
        'frostlab',
        'config',
        'diagnostic_aggregator.yaml'
    )
    sbg_diagnostics_config = os.path.join(
        '/home',
        'frostlab',
        'config',
        'sbg_diagnostics.yaml'
    )
    coug_fgo_dir = get_package_share_directory("coug_fgo")
    coug_fgo_launch_dir = os.path.join(coug_fgo_dir, "launch")

    verbose = "false"  # Default to 'false'
    param_file = '/home/frostlab/config/vehicle_params.yaml'

    if verbose == "true":
        output = 'screen'
    else:
        output = 'log'

    launch_actions = []
    namespace = LaunchConfiguration('namespace')
    sim = LaunchConfiguration('use_sim_time')

    launch_actions.extend([
        DeclareLaunchArgument('namespace', default_value='bluerov2'),
        DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true and  don't launch sensor drivers",
            ),
        launch_ros.actions.Node(
            package='dvl_a50',
            executable='dvl_a50_nav',
            name='dvl_nav_interface',
            parameters=[dvl_a50_config],
            namespace=LaunchConfiguration('namespace'),
            output='screen',
        ),
        launch_ros.actions.Node(
            package='dvl_a50',
            executable='dvl_a50_sensor',
            output='screen',
            namespace=LaunchConfiguration('namespace'),
            parameters=[dvl_a50_config],
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_zup_to_zdown',
            namespace=namespace,
            arguments=[
                '--x', '0', 
                '--y', '0', 
                '--z', '0', 
                '--qx', '1', 
                '--qy', '0', 
                '--qz', '0', 
                '--qw', '0',
                '--frame-id', 'bluerov2/dvl_odom',
                '--child-frame-id', 'bluerov2/dvl_odom_ned'],
            output='screen',
        ),

        # Map to DVL odom frame handling with GPS and IMU. 
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node_map',
        #     parameters=[param_file],
        #     namespace=namespace,
        #     remappings=[('odometry/filtered', 'odometry/global')]    
        # ),
        GroupAction(
            actions=[
                PushRosNamespace(namespace),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(coug_fgo_launch_dir, "blue_fgo.launch.py")
                    ),
                    launch_arguments={
                        "use_sim_time": sim,
                        "auv_ns": namespace,
                    }.items(),
                ),
            ]
        ),

        # launch_ros.actions.Node(
        #     package='seatrac',
        #     executable='modem_pinger',
        #     parameters=[param_file],
        #     namespace=LaunchConfiguration('namespace'),
        #     output=output,
        # ),
        Node(
            package='sensor_bringup',
            executable='gps_odom',
            parameters=[param_file],\
            namespace=LaunchConfiguration('namespace'),
            output=output,
            remappings=[('fix', 'imu/nav_sat_fix')],
        ),
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


        ################ Pressure sensor #########
        launch_ros.actions.Node(
            package='pressure_sensor',
            executable='pressure_pub',
            name='pressure_pub',
            parameters=[param_file],
            namespace=[LaunchConfiguration('namespace'), '/shallow'],
            output=output,
            condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
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
            condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
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
            executable='sbg_device',
            output='screen',
            namespace=LaunchConfiguration('namespace'),
            parameters=[sbg_config, sbg_datum],
            condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
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
            condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
        ),

        # Publish URDF static transforms (bluerov2/base_link -> sensor frames)
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # # Republish imu/odometry as odom -> bluerov2/base_link TF
        # launch_ros.actions.Node(
        #     package='sensor_bringup',
        #     executable='sbg_odom_tf',
        #     name='sbg_odom_tf',
        #     namespace=LaunchConfiguration('namespace'),
        # ),

        # map -> odom identity (datum = map origin; separate frames for future EKF)
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        ),

        # Continuous time-sync monitor. Publishes diagnostic_msgs on /diagnostics
        # and exposes the 'check_time_sync' std_srvs/Trigger service.
        launch_ros.actions.Node(
            package='topic_monitor',
            executable='topic_monitor_node',
            name='topic_monitor',
            namespace=LaunchConfiguration('namespace'),
            parameters=[time_sync_config],
            output=output,
        ),

        # Monitors SBG status/flag messages (EKF status, system power, GPS
        # accuracy/sats/diff age) and publishes diagnostic_msgs on /diagnostics.
        # What is checked is data-driven via config/sbg_diagnostics.yaml.
        launch_ros.actions.Node(
            package='sensor_bringup',
            executable='sbg_diagnostics',
            name='sbg_diagnostics',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'config_file': sbg_diagnostics_config}],
            output=output,
        ),

        # Aggregates /diagnostics into /diagnostics_agg so rqt_robot_monitor can
        # display the time-sync status tree.
        launch_ros.actions.Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='diagnostic_aggregator',
            parameters=[diagnostic_agg_config],
            output=output,
        ),

    ])

    return launch.LaunchDescription(launch_actions)