import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_cart_navigation = get_package_share_directory('hardware_navigation')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='localization.rviz',
        description='RViz config file'
    )

    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )

    localization_params_path = os.path.join(
        pkg_cart_navigation,
        'config',
        'amcl_localization.yaml'
    )

    map_file_path = os.path.join(
        pkg_cart_navigation,
        'maps',
        'map.yaml'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_cart_navigation, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': False},
        ]
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': localization_params_path,
            'map': map_file_path,
        }.items()
    )

    return LaunchDescription([
        rviz_launch_arg,
        rviz_config_arg,
        rviz_node,
        localization_launch
    ])
