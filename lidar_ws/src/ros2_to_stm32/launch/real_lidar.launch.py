import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('ros2_to_stm32'), 'config')
    return LaunchDescription([
        DeclareLaunchArgument('show_rviz', default_value='true'),
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='mid360_real_driver',
            output='screen',
            parameters=[os.path.join(config, 'livox_driver.yaml'), {
                'user_config_path': os.path.join(config, 'mid360_real.json'),
            }],
        ),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            output='screen',
            parameters=[os.path.join(config, 'fast_lio_mid360_real.yaml'),
                        {'use_sim_time': False}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(config, 'sensor.rviz')],
            condition=IfCondition(LaunchConfiguration('show_rviz')),
        ),
    ])
