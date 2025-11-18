from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    node_start_delay = LaunchConfiguration('node_start_delay')
    num_agents = LaunchConfiguration('num_agents')

    return LaunchDescription([
        DeclareLaunchArgument('node_start_delay', default_value='4.0'),
        DeclareLaunchArgument('num_agents', default_value='1'),

        # Include robo_description.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('cohan_sim_navigation'),
                    'launch',
                    'include',
                    'robo_description.py'
                ])
            ])
        ),

        # Include agent_tracking.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('cohan_sim_navigation'),
                    'launch',
                    'include',
                    'agent_tracking.py'
                ])
            ]),
            launch_arguments={'num_agents': num_agents}.items()
        ),

        # # Include move_base_nav.launch.py
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('cohan_sim_navigation'),
        #             'launch',
        #             'include',
        #             'move_base_nav.py'
        #         ])
        #     ]),
        #     launch_arguments={'node_start_delay': node_start_delay}.items()
        # ),

        # Include localization.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('cohan_sim_navigation'),
                    'launch',
                    'include',
                    'localization.py'
                ])
            ])
        ),

        # # Invisible humans detection node
        # Node(
        #     package='invisible_humans_detection',
        #     executable='invisible_humans_detection_node',
        #     name='map_scanner',
        #     output='screen'
        # )
    ])
