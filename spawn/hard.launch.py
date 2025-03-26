import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_cart_navigation = get_package_share_directory('hardware_navigation')

    urdf_file_path = PathJoinSubstitution([
        pkg_cart_navigation,
        "urdf",
        LaunchConfiguration('model')
    ])

    model_arg = DeclareLaunchArgument(
        'model', default_value='cart.urdf.xacro',
        description='URDF/Xacro file'
    )

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': Command(['xacro', urdf_file_path])}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            os.path.join(pkg_cart_navigation, 'config', 'ekf.yaml'),
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_cart_navigation, 'rviz', 'bot.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        model_arg,
        rviz_launch_arg,
        robot_state_publisher_node,
        ekf_node,
        rviz_node
    ])
