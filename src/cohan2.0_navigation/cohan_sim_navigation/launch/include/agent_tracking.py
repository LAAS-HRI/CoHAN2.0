from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare,FindPackagePrefix
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
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('ns'), "' == ''"])),
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
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('ns'), "' == ''"])),

        ),

        # With namespace
        Node(
            package='cohan_sim_navigation',
            executable='agents_bridge.py',
            name='agents',
            output='screen',
            arguments=[num_agents, ns],
            condition=UnlessCondition(PythonExpression(["'", LaunchConfiguration('ns'), "' == ''"]))
        ),
        Node(
            package='agent_path_prediction',
            executable='agent_path_predict',
            name='agent_path_predict',
            output='screen',
            parameters=[{
                'ns': ns,
                '~robot_frame_id': PathJoinSubstitution([
                    LaunchConfiguration('ns'),
                    'base_footprint'
                ]),
                'goals_file': PathJoinSubstitution([
                    FindPackageShare('agent_path_prediction'),
                    'cfg',
                    'goals_adream.yaml'
                ])
            }],
            remappings=[('map', '/map')],
            condition=UnlessCondition(PythonExpression(["'", LaunchConfiguration('ns'), "' == ''"])),
        )
    ])
