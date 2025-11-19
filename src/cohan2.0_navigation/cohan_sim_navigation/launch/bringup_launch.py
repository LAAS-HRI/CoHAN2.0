# This launch file is modified from nav2_bringup (Intel, Apache 2.0 License)
## Modified Nav2 launch file for CoHAN2.0 Navigation Stack##

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


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
    log_level = LaunchConfiguration('log_level')

    ## For Localization
    use_amcl = LaunchConfiguration('use_amcl')
    odom_frame = LaunchConfiguration('odom_frame')

    # Full path resolved at launch time
    map_path = PathJoinSubstitution([
        nav_dir,
        'maps',
        [map_name, TextSubstitution(text='.yaml')]
    ])

    ## Needed according to nav2 docs
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_path}

    # Only it applys when `use_namespace` is True.
    # '<robot_namespace>' keyword shall be replaced by 'ns' launch argument
    # in config file 'cohan2_multirobot_params.yaml' as a default & example.
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', namespace)},
        condition=IfCondition(use_namespace))

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
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
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_dir, 'params', 'cohan2_params_pr2.yaml'),
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

    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    use_amcl_arg = DeclareLaunchArgument(
        'use_amcl',
        default_value='false',
        description='Whether to start AMCL localization node')
    
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='odom frame id for fake_localization node')

    # Specify the actions
    bringup_nav2_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
            launch_arguments={'namespace': namespace,
                              'map': map_name,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'use_amcl': use_amcl,
                              'odom_frame': odom_frame,
                              'container_name': 'nav2_container'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(namespace_arg)
    ld.add_action(use_namespace_arg)
    ld.add_action(map_name_arg)
    ld.add_action(sim_time_arg)
    ld.add_action(params_file_arg)
    ld.add_action(autostart_arg)
    ld.add_action(use_composition_arg)
    ld.add_action(respawn_arg)
    ld.add_action(log_level_arg)
    ld.add_action(use_amcl_arg)
    ld.add_action(odom_frame_arg)

    # Add the actions to launch all files
    ld.add_action(bringup_nav2_group)

    return ld