from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    namespace = LaunchConfiguration('ns')
    use_namespace = LaunchConfiguration('use_namespace')
    num_agents = LaunchConfiguration('num_agents')
    use_sim_time = LaunchConfiguration('use_sim_time')

    namespace_arg = DeclareLaunchArgument(
        'ns',
        default_value='',
        description='Top-level namespace')
    
    use_namespace_arg = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the agents_bridge and agent_path_predict nodes')
    
    num_agents_arg = DeclareLaunchArgument(
        'num_agents',
        default_value='1',
        description='Number of agents the robot is navigating with (excluding the robot)')
    
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    agent_tracking_nodes = GroupAction(
        condition = UnlessCondition(use_namespace),
        actions=[   
            Node(
                package='cohan_sim_navigation',
                executable='agents_bridge.py',
                name='agents',
                output='screen',
                arguments=[num_agents],
                parameters=[{'use_sim_time': use_sim_time}]
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
                        'goals_adream.yaml']),
                    'use_sim_time': use_sim_time}]
                ),
        ]
    )

    namespaced_agent_tracking_nodes = GroupAction(
        condition = IfCondition(use_namespace),
        actions=[   
            Node(
                package='cohan_sim_navigation',
                executable='agents_bridge.py',
                name='agents',
                namespace=namespace,
                output='screen',
                arguments=[num_agents],
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            Node(
                package='agent_path_prediction',
                executable='agent_path_predict',
                name='agent_path_predict',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'robot_frame_id': PathJoinSubstitution([
                        namespace,
                        'base_footprint'
                    ]),
                    'goals_file': PathJoinSubstitution([
                        FindPackageShare('agent_path_prediction'),
                        'cfg',
                        'goals_adream.yaml'
                    ]),
                    'use_sim_time': use_sim_time
                }],
                remappings=[('map', '/map')]
            )
        ]
    )

    exit_event_handler = RegisterEventHandler(
        condition=UnlessCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=agent_tracking_nodes,
            on_exit=EmitEvent(event=Shutdown(reason='Robot Description exited'))))
    

    exit_event_handler_namespaced = RegisterEventHandler(
        condition=IfCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=namespaced_agent_tracking_nodes,
            on_exit=EmitEvent(event=Shutdown(reason='Robot Description exited')))) 

    ld = LaunchDescription()
    
    ## Add launch arguments
    ld.add_action(namespace_arg)
    ld.add_action(use_namespace_arg)
    ld.add_action(num_agents_arg)
    ld.add_action(sim_time_arg)

    ## Add agent tracking nodes
    ld.add_action(agent_tracking_nodes)
    ld.add_action(namespaced_agent_tracking_nodes)     

    ## Add exit event handlers
    ld.add_action(exit_event_handler)
    ld.add_action(exit_event_handler_namespaced)

    return ld