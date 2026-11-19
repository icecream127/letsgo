import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('ros2_to_stm32'), 'config')
    return LaunchDescription([
        DeclareLaunchArgument('show_rviz', default_value='true'),
        DeclareLaunchArgument('save_pcd', default_value='false'),
        DeclareLaunchArgument('map_file_path', default_value='./maps/default/cloud.pcd'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(config, 'robot.yaml')),
        DeclareLaunchArgument('lidar_network_file', default_value=os.path.join(config, 'mid360_real.json')),
        OpaqueFunction(function=real_lidar_nodes),
    ])


def real_lidar_nodes(context):
    config = os.path.join(get_package_share_directory('ros2_to_stm32'), 'config')
    default = os.path.join(config, 'robot.yaml')
    selected = LaunchConfiguration('params_file').perform(context)
    parameter_files = [default]
    if os.path.realpath(selected) != os.path.realpath(default):
        parameter_files.append(selected)
    return [
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='mid360_real_driver',
            output='screen',
            parameters=[*parameter_files, {
                'user_config_path': ParameterValue(LaunchConfiguration('lidar_network_file'), value_type=str),
            }],
        ),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laser_mapping',
            output='screen',
            parameters=[*parameter_files,
                        {'use_sim_time': False,
                         'pcd_save.pcd_save_en': ParameterValue(LaunchConfiguration('save_pcd'), value_type=bool),
                         'map_file_path': LaunchConfiguration('map_file_path')}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(config, 'sensor.rviz')],
            condition=IfCondition(LaunchConfiguration('show_rviz')),
        ),
    ]
