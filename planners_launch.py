from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    planner_params = [
        {
            "use_sim_time": True,
            "planner_server": {
                "planner_plugins": ["GridBased"],
                "GridBased": {
                    "plugin": "nav2_navfn_planner/NavfnPlanner",
                    "tolerance": 0.5,
                    "use_astar": False,
                }
            }
        }
    ]

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace="agent_planner",
        output="screen",
        parameters=planner_params,
        remappings=[('/agent_planner/map', '/map'),]
    )

    planner_server2 = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace="backoff_planner",
        output="screen",
        parameters=planner_params,
        remappings=[('/backoff_planner/map', '/map'),]
    )

    return LaunchDescription([
        planner_server,
        planner_server2
    ])
