import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction

def generate_launch_description():

    # Package name
    package_name='cart_navigation'

    # Launch configurations
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')

    # Path to default world 
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.world')

    # Launch Arguments
    declare_world = DeclareLaunchArgument(
        name='world', default_value=world_path,
        description='Full path to the world model file to load')
    
    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Opens RViz if set to True')

    # Launch Robot State Publisher
    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'cart.urdf.xacro')
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true', 'urdf': urdf_path}.items()
    )

    # Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': ['-r -s -v1 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': '-g '}.items()
    )

    # Robot spawner
    spawn_cart = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'cart', '-z', '0.2'],
        output='screen'
    )

    # Bridge between Gazebo and ROS2
    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ]
    )

    # Add SLAM Toolbox Node
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[('/scan', '/scan')]
    )

    return LaunchDescription([
        declare_rviz,
        declare_world,
        rsp,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        spawn_cart,
        slam_toolbox
    ])
