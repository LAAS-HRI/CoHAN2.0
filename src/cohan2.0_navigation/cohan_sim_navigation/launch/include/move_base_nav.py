import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    ns_arg = DeclareLaunchArgument('ns', default_value='')
    node_start_delay_arg = DeclareLaunchArgument('node_start_delay', default_value='4.0')
    bt_xml_arg = DeclareLaunchArgument('bt_xml', default_value='all_modes.xml')

    ns = LaunchConfiguration('ns')
    node_start_delay = LaunchConfiguration('node_start_delay')
    bt_xml = LaunchConfiguration('bt_xml')

    # Paths to config files
    package_share = FindPackageShare('cohan_sim_navigation')
    hateb_share = FindPackageShare('hateb_local_planner')

    move_base_params = PathJoinSubstitution([package_share, 'config', 'move_base_params.yaml'])
    global_costmap_params_robot = PathJoinSubstitution([package_share, 'config', 'robot', 'global_costmap_params.yaml'])
    local_costmap_params_robot = PathJoinSubstitution([package_share, 'config', 'robot', 'local_costmap_params.yaml'])
    hateb_params_robot = PathJoinSubstitution([package_share, 'config', 'robot', 'hateb_local_planner_params.yaml'])

    global_costmap_params_human = PathJoinSubstitution([package_share, 'config', 'human', 'global_costmap_params_human.yaml'])
    local_costmap_params_human = PathJoinSubstitution([package_share, 'config', 'human', 'local_costmap_params_human.yaml'])
    hateb_params_human = PathJoinSubstitution([package_share, 'config', 'human', 'hateb_local_planner_params_human.yaml'])

    bt_xml_path = PathJoinSubstitution([hateb_share, 'behavior_trees', bt_xml])

    # Default node (no namespace)
    default_node = TimerAction(
        period=node_start_delay,
        actions=[
            Node(
                package='move_base',
                executable='move_base',
                name='move_base',
                output='screen',
                parameters=[
                    {'base_global_planner': 'global_planner/GlobalPlanner'},
                    {'base_local_planner': 'hateb_local_planner/HATebLocalPlannerROS'},
                    {'GlobalPlanner/allow_unknown': True},
                    {'bt_xml_path': bt_xml_path},
                    {'use_simulated_fov': True},
                    move_base_params,
                    {'global_costmap': global_costmap_params_robot},
                    {'local_costmap': local_costmap_params_robot},
                    {'HATebLocalPlannerROS': hateb_params_robot},
                ],
                condition=UnlessCondition(ns)
            )
        ]
    )

    # Namespaced node
    ns_node = TimerAction(
        period=node_start_delay,
        actions=[
            Node(
                package='move_base',
                executable='move_base',
                name='move_base',
                output='screen',
                remappings=[('map', '/map')],
                parameters=[
                    {'ns': ns},
                    {'base_global_planner': 'global_planner/GlobalPlanner'},
                    {'base_local_planner': 'hateb_local_planner/HATebLocalPlannerROS'},
                    {'GlobalPlanner/allow_unknown': True},
                    {'bt_xml_path': bt_xml_path},
                    {'use_simulated_fov': True},
                    move_base_params,
                    {'global_costmap': global_costmap_params_human},
                    {'local_costmap': local_costmap_params_human},
                    {'HATebLocalPlannerROS': hateb_params_human},
                    {'global_costmap/robot_base_frame': ns + '/base_link'},
                    {'local_costmap/robot_base_frame': ns + '/base_link'},
                    {'local_costmap/global_frame': ns + '/odom'}
                ],
                condition=IfCondition(ns)
            )
        ]
    )

    return LaunchDescription([
        ns_arg,
        node_start_delay_arg,
        bt_xml_arg,
        default_node,
        ns_node
    ])
