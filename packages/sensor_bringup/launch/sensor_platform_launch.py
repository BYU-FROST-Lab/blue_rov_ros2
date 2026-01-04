import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	sbg_config = os.path.join(
		'/root',
		'config',
		'sbg_driver_params.yaml'
	)
	ntrip_config = os.path.join(
        '/root',
        'config',
        'ntrip_client_params.yaml'
    )
	nmea_config = os.path.join(
        '/root',
        'config',
        'nmea_gpsd_params.yaml'
    )
	ouster_config = os.path.join(
		'/root',
		'config',
		'lidar_driver_params.yaml'
	)

	ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')

	driver_launch_file_path = \
        Path(ouster_ros_pkg_dir) / 'launch' / 'driver.launch.py'
    
	driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(driver_launch_file_path)]),
        launch_arguments={
            'params_file': ouster_config,
            'ouster_ns': '',
            'os_driver_name': 'os_driver',
            'viz': 'False'
        }.items()
    )

	return LaunchDescription([
		DeclareLaunchArgument('namespace', default_value='/'),
		Node(
			package='sbg_driver',
		#	name='sbg_device_1',
			executable = 'sbg_device',
			output = 'screen',
			namespace=LaunchConfiguration('namespace'),
			parameters = [sbg_config]
		),
		Node(
			package='nmea_gpsd',
			executable='nmea_gpsd_udp',
			output='screen',
			namespace=LaunchConfiguration('namespace'),
			parameters=[nmea_config]
        ),
		Node(
            package='ntrip_client',
            executable='ntrip_ros.py',
            name='ntrip_client',
            namespace=LaunchConfiguration('namespace'),
            parameters=[ntrip_config],
        ),
		# driver_launch
		
	])
