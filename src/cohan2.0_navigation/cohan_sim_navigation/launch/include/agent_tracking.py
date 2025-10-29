from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ns = LaunchConfiguration('ns')
    num_agents = LaunchConfiguration('num_agents')

    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value=''),
        DeclareLaunchArgument('num_agents', default_value='1'),

        # Without namespace
        Node(
            package='cohan_sim_navigation',
            executable='agents_bridge.py',
            name='agents',
            output='screen',
            arguments=[num_agents],
            condition=IfCondition(ns.perform(lambda x: x == ''))
        ),
        Node(
            package='agent_path_prediction',
            executable='agent_path_predict',
            name='agent_path_predict',
            output='screen',
            parameters=[{
                'goals_file': PathJoinSubstitution([
                    FindPackageShare('agent_path_prediction'),
                    'cfg',
                    'goals_adream.yaml'
                ])
            }],
            condition=IfCondition(ns.perform(lambda x: x == ''))
        ),

        # With namespace
        Node(
            package='cohan_sim_navigation',
            executable='agents_bridge.py',
            name='agents',
            output='screen',
            arguments=[num_agents, ns],
            condition=UnlessCondition(ns.perform(lambda x: x == ''))
        ),
        Node(
            package='agent_path_prediction',
            executable='agent_path_predict',
            name='agent_path_predict',
            output='screen',
            parameters=[{
                'ns': ns,
                '~robot_frame_id': PathJoinSubstitution([ns, 'base_footprint']),
                'goals_file': PathJoinSubstitution([
                    FindPackageShare('agent_path_prediction'),
                    'cfg',
                    'goals_adream.yaml'
                ])
            }],
            remappings=[('map', '/map')],
            condition=UnlessCondition(ns.perform(lambda x: x == ''))
        )
    ])
