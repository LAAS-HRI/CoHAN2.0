## Planners for agent_path_planning and backoff_recovery

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true')

    planner_params = [
        {
            'use_sim_time': use_sim_time,
            'planner_plugins': ["GridBased"],
            'expected_planner_frequency': 5.0,
            'GridBased': {
                'plugin': "nav2_navfn_planner/NavfnPlanner",
                'tolerance': 0.5,
                'use_astar': False,
                'allow_unknown': True
            }
        }
    ]

    launch_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_planner',
                executable='planner_server',
                namespace='agent_planner',
                output='screen',
                parameters=planner_params,
                remappings=[('/agent_planner/map', '/map'),]
            ),

            Node(
                package='nav2_planner',
                executable='planner_server',
                namespace='backoff_planner',
                output='screen',
                parameters=planner_params,
                remappings=[('/backoff_planner/map', '/map'),]
            ),
            ## Lifecycle activation with delay --> Nav2_lifecycle_manager does not work well here
            Node(
                package='cohan_sim_navigation',
                executable='activate_lifecylenodes_with_delay.py',
                name='activate_planners_client',
                output='screen',
                arguments=['1.0', 'agent_planner/planner_server', 'backoff_planner/planner_server']
            ),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(launch_nodes) 
    
    return ld
