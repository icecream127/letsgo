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
            get_package_share_directory('ros2_to_stm32'), 'config', 'robot.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('scan_min_height', default_value=''),
        DeclareLaunchArgument('scan_max_height', default_value=''),
        OpaqueFunction(function=navigation_nodes),
    ])


def navigation_nodes(context):
    backend = LaunchConfiguration('localization').perform(context)
    gicp = backend == 'gicp'
    share = get_package_share_directory('ros2_to_stm32')
    selected = LaunchConfiguration('params_file').perform(context)
    default = os.path.join(share, 'config', 'robot.yaml')
    parameter_files = [default]
    if os.path.realpath(selected) != os.path.realpath(default):
        parameter_files.append(selected)
    scan_overrides = {}
    for key in ('min_height', 'max_height'):
        value = LaunchConfiguration(f'scan_{key}').perform(context)
        if value:
            scan_overrides[key] = float(value)
    sim = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    common = {'use_sim_time': sim}
    actions = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'real_lidar.launch.py')),
            condition=IfCondition(LaunchConfiguration('sensors')),
            launch_arguments={'show_rviz': 'false', 'save_pcd': 'false',
                              'params_file': selected}.items()),
        Node(package='ros2_to_stm32', executable='nav_adapter', output='screen',
             parameters=[*parameter_files, common, {'use_gicp': gicp}]),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='pointcloud_to_laserscan', output='screen',
             remappings=[('cloud_in', '/cloud_registered_nav'), ('scan', '/scan')],
             parameters=[*parameter_files, common, scan_overrides]),
        Node(package='ros2_to_stm32', executable='nav_cmd_gate', output='screen', parameters=[*parameter_files, common, {'use_gicp': gicp}]),
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
        actions.append(Node(package='go2_real_localization', executable='gicp_localizer',
            name='gicp_localizer', output='screen', parameters=[*parameter_files, common, {
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
            output='screen', parameters=[*parameter_files, overrides], remappings=[('cmd_vel', '/cmd_vel_nav')]))
    for group, names in [('localization', localization_names),
                         ('navigation', navigation_names)]:
        actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_' + group, output='screen', parameters=[common, {
                'autostart': True, 'node_names': names}]))
    actions.append(Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
        parameters=[common], remappings=[('/initialpose', '/gicp/initialpose')] if gicp else []))
    return actions
