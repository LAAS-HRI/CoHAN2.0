# This launch file is modified from nav2_bringup (Intel, Apache 2.0 License)
## All in one launch for CoHAN2.0 navigation with PR2 using CoHAN SIM

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    nav_dir = get_package_share_directory('cohan_sim_navigation')
    launch_dir = os.path.join(nav_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('ns')
    use_namespace = LaunchConfiguration('use_namespace')
    map_name = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    num_agents = LaunchConfiguration('num_agents')
    use_amcl = LaunchConfiguration('use_amcl')
    odom_frame = LaunchConfiguration('odom_frame')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')

    # Required according to nav2 docs
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    namespace_arg = DeclareLaunchArgument(
        'ns',
        default_value='',
        description='Top-level namespace')

    use_namespace_arg = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    map_name_arg = DeclareLaunchArgument(
        'map',
        default_value='laas',
        description='Name of the map to load')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_dir, 'config', 'cohan2_params_pr2.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    use_composition_arg = DeclareLaunchArgument(
        'use_composition', default_value='true',
        description='Whether to use composed bringup')

    respawn_arg = DeclareLaunchArgument(
        'use_respawn', default_value='false',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(nav_dir, 'rviz', 'cohan.rviz'),
        description='Full path to the RVIZ config file to use')

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Whether to start the simulator')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RVIZ')
    
    use_amcl_arg = DeclareLaunchArgument(
        'use_amcl',
        default_value='false',
        description='Whether to start AMCL localization node')
    
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='odom frame id for fake_localization node')

    cohan_sim_node = ExecuteProcess(
        condition=UnlessCondition(use_gui),
        cmd=[
                PathJoinSubstitution([
                    ('cohan_sim'),
                    'lib',
                    'cohan_sim',
                    'simros_node'
                ]),
                PathJoinSubstitution([
                    FindPackageShare('cohan_sim_navigation'),
                    'maps',
                    [map_name, '.yaml']
                ])
            ],
        output='screen',
        name='cohan_sim')
    
    cohan_sim_node_gui =  ExecuteProcess(
            condition=IfCondition(use_gui),
            cmd=[
                    PathJoinSubstitution([
                        FindPackagePrefix('cohan_sim'),
                        'lib',
                        'cohan_sim',
                        'simros_node'
                    ]),
                    '-g',
                    PathJoinSubstitution([
                        FindPackageShare('cohan_sim_navigation'),
                        'maps',
                        [map_name, '.yaml']
                    ])
                ],
            output='screen',
            name='cohan_sim')

    pr2_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'pr2_description_launch.py')),
        launch_arguments={'ns': namespace,
                          'use_namespace': use_namespace,
                          'use_sim_time': use_sim_time,
                          'use_respawn': use_respawn}.items())
    
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz2_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'ns': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())
    
    
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'ns': namespace,
                          'use_namespace': use_namespace,
                          'map': map_name,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart,
                          'use_amcl': use_amcl,
                          'odom_frame': odom_frame,   
                          'use_composition': use_composition,
                          'use_respawn': use_respawn}.items())
    
    agent_tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'agent_tracking_launch.py')),
        launch_arguments={'ns': namespace,
                          'use_namespace': use_namespace,
                          'num_agents': num_agents,
                          'use_sim_time': use_sim_time}.items())

    # Exit event handlers to shutdown launch when simulator exits
    exit_event_handler_sim = RegisterEventHandler(
        condition=UnlessCondition(use_gui),
        event_handler=OnProcessExit(
            target_action=cohan_sim_node,
            on_exit=EmitEvent(event=Shutdown(reason='Simulator exited'))))

    exit_event_handler_sim_gui = RegisterEventHandler(
        condition=IfCondition(use_gui),
        event_handler=OnProcessExit(
            target_action=cohan_sim_node_gui,
            on_exit=EmitEvent(event=Shutdown(reason='Simulator with GUI exited'))))

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(namespace_arg)
    ld.add_action(use_namespace_arg)
    ld.add_action(map_name_arg)
    ld.add_action(sim_time_arg)
    ld.add_action(params_file_arg)
    ld.add_action(autostart_arg)
    ld.add_action(use_composition_arg)
    ld.add_action(respawn_arg)
    ld.add_action(rviz_config_file_arg)
    ld.add_action(use_gui_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(use_amcl_arg)
    ld.add_action(odom_frame_arg)

    # Add any conditioned actions or included launch descriptions
    ld.add_action(cohan_sim_node)
    ld.add_action(cohan_sim_node_gui)
    ld.add_action(pr2_description_launch)
    ld.add_action(rviz_launch)
    ld.add_action(bringup_launch)
    # ld.add_action(agent_tracking_launch)

    # Add exit event handlers
    ld.add_action(exit_event_handler_sim)
    ld.add_action(exit_event_handler_sim_gui)

    return ld