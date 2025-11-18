from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    map_name = LaunchConfiguration('map_name')
    gui = LaunchConfiguration('gui')
    
    # Use ExecuteProcess instead of Node to avoid ROS argument interference
    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='laas'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
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
        )
    ])