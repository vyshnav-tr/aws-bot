from launch import LaunchDescription
from launch.actions import (TimerAction, IncludeLaunchDescription,
                             AppendEnvironmentVariable, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    world_path = os.path.join(
        get_package_share_directory('robot_description'),
        'world', 'small_warehouse.sdf'
    )

    pkg_name = 'robot_description'
    pkg_dir  = get_package_share_directory(pkg_name)

    # ── Spawn position: set z = actual_floor_height + 0.05
    # Run the grep commands above to confirm your floor height first!
    SPAWN_X = '0.0'
    SPAWN_Y = '1.0'
    SPAWN_Z = '1'   # ← CHANGE THIS to your actual 2nd floor z + 0.05

    xacro_file = os.path.join(pkg_dir, 'urdf', 'bcr_bot.xacro')
    robot_description = {
        'robot_description': xacro.process_file(xacro_file ,mappings={
            'sim_gazebo':           'true',
            'two_d_lidar_enabled':  'true',   # set as you need
            'camera_enabled':       'true',
            'robot_namespace':      '',
            'odometry_source':      'world',
        }).toxml()  # type: ignore
    }
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    set_model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(pkg_dir, 'models')
    )
    # ── Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path,'verbose': 'true'}.items()
    )

    # ── Robot State Publisher: start immediately so /robot_description is ready
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0,
        }]
    )

    # ── Spawn: wait 8s for Gazebo + world to fully load before spawning
    spawn_entity = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'cleaner_robot',
                    '-x', SPAWN_X,
                    '-y', SPAWN_Y,
                    '-z', SPAWN_Z,
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        set_model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])