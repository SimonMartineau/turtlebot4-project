import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    lidar_config = os.path.join(
        get_package_share_directory('turtlebot4_bringup'),
        'config',
        'lidar_noise_params.yaml'
    )

    visual_config = os.path.join(
        get_package_share_directory('turtlebot4_bringup'),
        'config',
        'visual_noise_params.yaml'
    )

    dust_noise_node = Node(
    package="turtlebot4_slam_noise",
    executable="dust_noise",
    name="dust_noise_node",
    parameters=[lidar_config]
    )

    visual_noise_node = Node(
    package="turtlebot4_slam_noise",
    executable="visual_noise",
    name="visual_noise_node",
    parameters=[visual_config]
    )

    ld.add_action(dust_noise_node)
    ld.add_action(visual_noise_node)
    return ld