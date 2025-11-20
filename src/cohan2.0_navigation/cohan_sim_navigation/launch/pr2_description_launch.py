## PR2 Robot Description Launch File for RVIZ visualization

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def generate_launch_description():
    ## Get the navigation directory with error handling
    try:
        nav_dir = get_package_share_directory('cohan_sim_navigation')
    except PackageNotFoundError:
        print("[WARNING] Package 'cohan_sim_navigation' not found. Skipping robot description nodes.")
        nav_dir = None

    robot_description = None
    if nav_dir:
        urdf_path = os.path.join(nav_dir, 'robots', 'pr2.urdf')
        if os.path.exists(urdf_path):
            with open(urdf_path, 'r') as urdf_file:
                robot_description = urdf_file.read()
        else:
            print(f"[WARNING] URDF file not found at {urdf_path}, skipping robot description nodes.")

    namespace = LaunchConfiguration('ns')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')


    namespace_arg = DeclareLaunchArgument(
        'ns',
        default_value='',
        description='Top-level namespace')
    
    use_namespace_arg = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    respawn_arg = DeclareLaunchArgument(
        'use_respawn', default_value='false',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level')

    pr2_description_launch = GroupAction(
        condition = UnlessCondition(use_namespace),
        actions=[] if robot_description is None else [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
                ),
            Node(
                package='cohan_sim_navigation',
                executable='publish_joints.py',
                name='sim_joints',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
                ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
                parameters=[{'source_list': ['/cohan_sim_joint_states'], 'rate': 50}]
            )
        ]
    )
    
    namespaced_pr2_description_launch = GroupAction(
        condition = IfCondition(use_namespace),
        actions=[] if robot_description is None else [
            Node(
                condition=IfCondition(use_namespace),
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=namespace,
                respawn=use_respawn,
                respawn_delay=2.0,
                output='screen',
                parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
                remappings=[('/tf', 'tf'),
                            ('/tf_static', 'tf_static'),
                            ('/joint_states', 'joint_states')],
                arguments=['--ros-args', '--log-level', log_level]),
            Node(
                package='cohan_sim_navigation',
                executable='publish_joints.py',
                name='sim_joints',
                respawn=use_respawn,
                respawn_delay=2.0,
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                namespace=namespace,
                respawn=use_respawn,
                respawn_delay=2.0,
                output='screen',
                parameters=[{'source_list': ['/cohan_sim_joint_states'], 'rate': 50}]
            )
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(namespace_arg)
    ld.add_action(use_namespace_arg)
    ld.add_action(sim_time_arg)
    ld.add_action(respawn_arg)
    ld.add_action(log_level_arg)

    # Add any conditioned actions
    ld.add_action(pr2_description_launch)
    ld.add_action(namespaced_pr2_description_launch)

    return ld