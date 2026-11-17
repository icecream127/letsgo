"""Official Nav2 nodes with FAST-LIO adapters; no serial or motor nodes."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='amcl', choices=['amcl', 'gicp']),
        DeclareLaunchArgument('pcd', default_value=''),
        DeclareLaunchArgument('gicp_seed', default_value=''),
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('ros2_to_stm32'), 'config', 'nav2.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('scan_min_height', default_value='-0.3'),
        DeclareLaunchArgument('scan_max_height', default_value='1.0'),
        OpaqueFunction(function=navigation_nodes),
    ])


def navigation_nodes(context):
    backend = LaunchConfiguration('localization').perform(context)
    gicp = backend == 'gicp'
    share = get_package_share_directory('ros2_to_stm32')
    config = LaunchConfiguration('params_file')
    sim = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    common = {'use_sim_time': sim}
    actions = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'real_lidar.launch.py')),
            condition=IfCondition(LaunchConfiguration('sensors')),
            launch_arguments={'show_rviz': 'false', 'save_pcd': 'false'}.items()),
        Node(package='ros2_to_stm32', executable='nav_adapter', output='screen',
             parameters=[config, common, {'use_gicp': gicp}]),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node', output='screen',
             remappings=[('cloud_in', '/cloud_registered_nav'), ('scan', '/scan')], parameters=[common, {
                 'target_frame': 'nav_scan', 'transform_tolerance': 0.2, 'queue_size': 20,
                 'min_height': ParameterValue(LaunchConfiguration('scan_min_height'), value_type=float),
                 'max_height': ParameterValue(LaunchConfiguration('scan_max_height'), value_type=float),
                 'angle_min': -3.141592653589793, 'angle_max': 3.141592653589793,
                 'angle_increment': 0.00872664626, 'scan_time': 0.1,
                 'range_min': 0.8, 'range_max': 12.0, 'use_inf': True}]),
        Node(package='ros2_to_stm32', executable='nav_cmd_gate', output='screen', parameters=[common, {'use_gicp': gicp}]),
    ]
    nodes = [('nav2_map_server', 'map_server'),
             ('nav2_controller', 'controller_server'), ('nav2_planner', 'planner_server'),
             ('nav2_smoother', 'smoother_server'), ('nav2_behaviors', 'behavior_server'),
             ('nav2_bt_navigator', 'bt_navigator')]
    localization_names = ['map_server']
    navigation_names = [name for _, name in nodes[1:]]
    if gicp:
        pcd = LaunchConfiguration('pcd').perform(context)
        seed = LaunchConfiguration('gicp_seed').perform(context)
        if not os.path.isfile(pcd) or not seed:
            raise ValueError('GICP requires an existing pcd and gicp_seed; use run/navigation_gicp.sh')
        gicp_share = get_package_share_directory('go2_real_localization')
        actions.append(Node(package='go2_real_localization', executable='gicp_localizer',
            name='gicp_localizer', output='screen', parameters=[os.path.join(gicp_share, 'config', 'gicp.yaml'), common, {
                'map_path': pcd, 'initial_pose_enabled': True,
                'initial_pose': ParameterValue(LaunchConfiguration('gicp_seed'), value_type=list[float]),
            }], remappings=[('/initialpose', '/gicp/initialpose')]))
    else:
        nodes.append(('nav2_amcl', 'amcl'))
        localization_names.append('amcl')
    for package, executable in nodes:
        overrides = dict(common)
        if executable == 'map_server': overrides['yaml_filename'] = LaunchConfiguration('map')
        actions.append(Node(package=package, executable=executable, name=executable,
            output='screen', parameters=[config, overrides], remappings=[('cmd_vel', '/cmd_vel_nav')]))
    for group, names in [('localization', localization_names),
                         ('navigation', navigation_names)]:
        actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_' + group, output='screen', parameters=[common, {
                'autostart': True, 'node_names': names}]))
    actions.append(Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
        parameters=[common], remappings=[('/initialpose', '/gicp/initialpose')] if gicp else []))
    return actions
