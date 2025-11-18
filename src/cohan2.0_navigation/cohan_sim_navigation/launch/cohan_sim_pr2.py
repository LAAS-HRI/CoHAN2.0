from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    gui = LaunchConfiguration('gui')
    node_start_delay = LaunchConfiguration('node_start_delay')
    num_agents = LaunchConfiguration('num_agents')
    map_name = LaunchConfiguration('map_name')
    ns = LaunchConfiguration('ns')

    from launch.substitutions import TextSubstitution
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('node_start_delay', default_value='2.0'),
        DeclareLaunchArgument('num_agents', default_value='2'),
        DeclareLaunchArgument('map_name', default_value='laas'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('ns', default_value=''),

        # Launch simulator without GUI
        ExecuteProcess(
            condition=UnlessCondition(gui),
            cmd=[
                PathJoinSubstitution([
                    FindPackagePrefix('cohan_sim'),
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
            name='cohan_sim'
        ),
        
        # Launch simulator with GUI
        ExecuteProcess(
            condition=IfCondition(gui),
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
            name='cohan_sim'
        ),

        # Start map_server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            # Provide the map filename via the yaml_filename parameter (required by nav2 map_server)
            parameters=[
                {
                    'yaml_filename': PathJoinSubstitution([
                        FindPackageShare('cohan_sim_navigation'),
                        'maps',
                        [map_name, '.yaml']
                    ]),
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }
            ],
            output='screen'
        ),

        # Include robot_only.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('cohan_sim_navigation'),
                    'launch',
                    'include',
                    'robot_only.py'
                ])
            ]),
            launch_arguments={
                'node_start_delay': node_start_delay,
                'num_agents': num_agents
            }.items()
        ),

        # # Start RViz
        # Node(
        #     package='rviz2',  # ROS2 package
        #     executable='rviz2',
        #     name='rviz',
        #     arguments=[
        #         '-d',
        #         PathJoinSubstitution([
        #             FindPackageShare('cohan_sim_navigation'),
        #             'rviz',
        #             'cohan_sim.rviz'
        #         ])
        #     ],
        #     output='screen'
        # )
    ])
