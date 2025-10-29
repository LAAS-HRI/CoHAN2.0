from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os

def launch_setup(context, *args, **kwargs):
    try:
        cohan_sim_navigation_dir = get_package_share_directory('cohan_sim_navigation')
    except PackageNotFoundError:
        print("[WARNING] Package 'cohan_sim_navigation' not found. Skipping related nodes.")
        cohan_sim_navigation_dir = None

    nodes = []

    # robot_state_publisher with URDF if package exists
    if cohan_sim_navigation_dir:
        urdf_path = os.path.join(cohan_sim_navigation_dir, 'robots', 'pr2.urdf')
        if os.path.exists(urdf_path):
            with open(urdf_path, 'r') as urdf_file:
                robot_description = urdf_file.read()
            
            nodes.append(Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}]
            ))

            nodes.append(Node(
                package='cohan_sim_navigation',
                executable='publish_joints.py',
                name='sim_joints',
                output='screen'
            ))

            nodes.append(Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
                parameters=[{'source_list': ['/cohan_sim_joint_states'], 'rate': 50}]
            ))
        else:
            print(f"[WARNING] URDF file not found at {urdf_path}, skipping robot_state_publisher.")
    
    return nodes

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
