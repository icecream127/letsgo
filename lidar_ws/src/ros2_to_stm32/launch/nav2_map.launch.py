"""Load a saved occupancy map only; no localization, controller or serial nodes."""
from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('ros2_to_stm32'), 'config', 'robot.yaml')),
        DeclareLaunchArgument('map', description='Absolute path to map.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[LaunchConfiguration('params_file'), {'yaml_filename': LaunchConfiguration('map'),
                                          'frame_id': 'map', 'use_sim_time': sim}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', output='screen', parameters=[{
                 'use_sim_time': sim, 'autostart': True, 'node_names': ['map_server']}]),
    ])
