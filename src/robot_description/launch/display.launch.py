import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'robot_description'
    pkg_dir = get_package_share_directory(pkg_name)
    rviz_config_file = os.path.join(pkg_dir, 'rviz',   'entire_setup.rviz')
    nav2_params_file = os.path.join(pkg_dir, 'params', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = os.path.join(pkg_dir, 'map', 'my_map.yaml')

    # ── Nav2 bringup ──────────────────────────────────────────────────────────
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map':         map_file,
            'use_sim_time': 'True',
            'params_file': nav2_params_file,
            'autostart':   'True',
        }.items()
    )

    # ── Route server ──────────────────────────────────────────────────────────
    route_server = Node(
        package='nav2_route',
        executable='route_server',
        name='route_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    
    filter_mask_server = Node(
    package='nav2_map_server',
    executable='map_server',
    name='filter_mask_server',
    output='screen',
    parameters=[nav2_params_file]   # ← reads from yaml, not inline
)
    
    keepout_mask_server = Node(
    package='nav2_map_server',
    executable='map_server',
    name='keepout_mask_server',
    output='screen',
    parameters=[nav2_params_file]
)

    keepout_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='keepout_filter_info_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[nav2_params_file]   # ← reads from yaml, not inline
    )
    
    filter_lifecycle_manager = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_costmap_filters',
    output='screen',
    parameters=[{
        'use_sim_time': True,
        'autostart': True,
        'bond_timeout': 10.0,
        'node_names': [
            'filter_mask_server',
            'costmap_filter_info_server',
            'keepout_mask_server',          # ← add
            'keepout_filter_info_server',
            'route_server'
        ],
    }]
)


    

    # ── RViz ──────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        rviz_node,
        filter_mask_server,           
        costmap_filter_info_server,
        keepout_mask_server,           # ← add
        keepout_filter_info_server,
        filter_lifecycle_manager,
        nav2_bringup,
        route_server,
    ])