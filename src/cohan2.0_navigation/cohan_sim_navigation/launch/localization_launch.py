# This launch file is modified from nav2_bringup (Intel, Apache 2.0 License)
## Modified Nav2 launch file for CoHAN2.0 Navigation Stack##

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    ## Get the navigation directory
    nav_dir = get_package_share_directory('cohan_sim_navigation')

    ## Common for all nodes
    namespace = LaunchConfiguration('ns')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')


    ## For map and localization
    map_name = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_amcl = LaunchConfiguration('use_amcl')
    odom_frame = LaunchConfiguration('odom_frame')

    # Full path resolved at launch time
    map_path = PathJoinSubstitution([
        nav_dir,
        'maps',
        [map_name, TextSubstitution(text='.yaml')]
    ])

    ## Lifecycle nodes - conditionally include amcl
    lifecycle_nodes_with_amcl = ['map_server', 'amcl']
    lifecycle_nodes_without_amcl = ['map_server']
    autostart = LaunchConfiguration('autostart')

    ## Needed accrording to nav2 docs
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    ## Create our own temporary YAML files that include substitutions (similar to nav2_bringup)
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_path}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    
    ## Declare the launch arguments specific to this file
    namespace_arg = DeclareLaunchArgument(
        'ns',
        default_value='',
        description='Top-level namespace')
    
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')
    
    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='false',
        description='Use composed bringup if true')
    
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='cohan2_container',
        description='The name of the container that nodes will load into if use_composition is true')
    
    respawn_arg = DeclareLaunchArgument(
        'use_respawn', default_value='false',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    map_name_arg = DeclareLaunchArgument(
        'map',
        default_value='laas',
        description='Name of the map to load')
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_dir, 'config', 'cohan2_params_pr2.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    use_amcl_arg = DeclareLaunchArgument(
        'use_amcl',
        default_value='False',
        description='Whether to start AMCL localization node')
    
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='odom frame id for fake_localization node')

    ## For logging immediately to console
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    launch_nodes = GroupAction(
        condition = UnlessCondition(use_composition),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                condition=IfCondition(use_amcl)
                ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='fake_localization',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['0', '0', '0', '0', '0', '0', 'map', odom_frame],
                remappings=remappings,
                condition=UnlessCondition(use_amcl)
                ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes_with_amcl}],
                condition=IfCondition(use_amcl)),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes_without_amcl}],
                condition=UnlessCondition(use_amcl))
        ]

    )

    launch_composable_nodes_with_amcl = GroupAction(
        condition=IfCondition(use_composition),
        actions= [
            LoadComposableNodes(
                condition=IfCondition(use_amcl),
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='map_server',
                        parameters=[configured_params],
                        remappings=remappings),
                    ComposableNode(
                        package='nav2_amcl',
                        plugin='nav2_amcl::AmclNode',
                        name='amcl',
                        parameters=[configured_params],
                        remappings=remappings),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_localization',
                        parameters=[{'use_sim_time': use_sim_time,
                                        'autostart': autostart,
                                        'node_names': lifecycle_nodes_with_amcl}]),
                ],
            )
        ]
    )

    launch_composable_nodes_without_amcl = GroupAction(
        condition=IfCondition(use_composition),
        actions= [
            ## Static transform publisher does not have a composable node version
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='fake_localization',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['0', '0', '0', '0', '0', '0', 'map', odom_frame],
                remappings=remappings,
                condition=UnlessCondition(use_amcl)
                ),
            LoadComposableNodes(
                condition=UnlessCondition(use_amcl),
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='map_server',
                        parameters=[configured_params],
                        remappings=remappings),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_localization',
                        parameters=[{'use_sim_time': use_sim_time,
                                        'autostart': autostart,
                                        'node_names': lifecycle_nodes_without_amcl}]),
                ],
            )
        ]
    )

    ld = LaunchDescription()

    ## Set env var for logging
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(namespace_arg)
    ld.add_action(sim_time_arg)
    ld.add_action(use_composition_arg)
    ld.add_action(container_name_arg)
    ld.add_action(respawn_arg)
    ld.add_action(log_level_arg)
    ld.add_action(map_name_arg)
    ld.add_action(params_file_arg)
    ld.add_action(use_amcl_arg)
    ld.add_action(odom_frame_arg)

    # Add the actions to launch the nodes
    ld.add_action(launch_nodes)
    ld.add_action(launch_composable_nodes_with_amcl)
    ld.add_action(launch_composable_nodes_without_amcl)

    return ld