from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    ns = LaunchConfiguration('ns', default='')

    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value=''),

        # Case when namespace is empty
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('ns'), "' == ''"]))
        ),

        # Case when namespace is not empty
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_ns_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', [ns, '/odom']],
            output='screen',
            condition=UnlessCondition(PythonExpression(["'", LaunchConfiguration('ns'), "' == ''"]))
        ),
    ])
