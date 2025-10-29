from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    gui = LaunchConfiguration('gui')
    node_start_delay = LaunchConfiguration('node_start_delay')
    num_agents = LaunchConfiguration('num_agents')
    map_name = LaunchConfiguration('map_name')

    from launch.substitutions import TextSubstitution
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('node_start_delay', default_value='2.0'),
        DeclareLaunchArgument('num_agents', default_value='1'),
        DeclareLaunchArgument('map_name', default_value='laas'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Launch simulator with GUI
        GroupAction(
            condition=IfCondition(gui),
            actions=[
                Node(
                    package='cohan_sim',
                    executable='simros_node',
                    name='libsim',
                    arguments=[
                        '-g',
                        PathJoinSubstitution([
                            FindPackageShare('cohan_sim_navigation'),
                            'maps',
                            TextSubstitution(text=''),
                            map_name,
                            TextSubstitution(text='.yaml')
                        ])
                    ],
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                    output='screen'
                )
            ]
        ),

        # Launch simulator without GUI
        GroupAction(
            condition=UnlessCondition(gui),
            actions=[
                Node(
                    package='cohan_sim',
                    executable='simros_node',
                    name='libsim',
                    arguments=[
                        PathJoinSubstitution([
                            FindPackageShare('cohan_sim_navigation'),
                            'maps',
                            TextSubstitution(text=''),
                            map_name,
                            TextSubstitution(text='.yaml')
                        ])
                    ],
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                    output='screen'
                )
            ]
        ),

        # Start map_server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            arguments=[
                PathJoinSubstitution([
                    FindPackageShare('cohan_sim_navigation'),
                    'maps',
                    TextSubstitution(text=''),
                    map_name,
                    TextSubstitution(text='.yaml')
                ])
            ],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),

        # Include robot_only.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('cohan_sim_navigation'),
                    'launch',
                    'include',
                    'robot_only.launch.py'
                ])
            ]),
            launch_arguments={
                'node_start_delay': node_start_delay,
                'num_agents': num_agents
            }.items()
        ),

        # Start RViz
        Node(
            package='rviz2',  # ROS2 package
            executable='rviz2',
            name='rviz',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('cohan_sim_navigation'),
                    'rviz',
                    'cohan_sim.rviz'
                ])
            ],
            output='screen'
        )
    ])
