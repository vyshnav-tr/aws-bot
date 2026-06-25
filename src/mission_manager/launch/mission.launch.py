from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


docking_params = os.path.join(
    get_package_share_directory('robot_description'),
    'params',
    'docking_server.yaml'
)

def generate_launch_description():
    return LaunchDescription([
        Node(
            name='aruco_tracker_autostart',
            package='aruco_opencv',
            executable='aruco_tracker_autostart',
            parameters=[{'cam_base_topic': '/kinect_camera/image_raw',
                         'marker_size': 0.15}],
            output='screen',
        ),
        Node(
            name='aruco_pose_bridge',
            package='robot_gazebo',
            executable='aruco_pose_bridge',
            output='screen',
        ),
        Node(
            name='battery_node',
            package='mission_manager',
            executable='battery_node',
            output='screen',
        ),
        Node(
            name='docking_server',
            package='opennav_docking',
            executable='opennav_docking',
            parameters=[
                {'use_sim_time': True},
                docking_params
            ],
                output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_docking',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'bond_timeout': 10.0,
                'node_names': [
                    'docking_server',
                ],
            }]
        ),
        Node(
            name='auto_dock',
            package='mission_manager',
            executable='auto_dock',
                output='screen',
        ),
        # More Nodes!
    ])