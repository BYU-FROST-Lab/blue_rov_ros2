import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
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
		driver_launch
		
	])
