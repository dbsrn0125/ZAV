import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('mvs_ros2_driver')
    default_config = os.path.join(pkg_dir, 'config', 'zenith_camera.yaml')

    config_file = LaunchConfiguration('config_file')

    declare_config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to camera config YAML file'
    )

    camera_node = Node(
        package='mvs_ros2_driver',
        executable='mvs_camera_node',
        name='mvs_camera',
        arguments=[config_file, '--ros-args', '--log-level', 'info'],
        respawn=True,
        respawn_delay=2.0,
        output='screen'
    )

    return LaunchDescription([
        declare_config_arg,
        camera_node
    ])
