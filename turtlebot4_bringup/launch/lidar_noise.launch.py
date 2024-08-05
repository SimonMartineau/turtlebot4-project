import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('turtlebot4_bringup'),
        'config',
        'params.yaml'
    )

    dust_noise_node = Node(
    package="turtlebot4_slam_noise",
    executable="dust_noise",
    name="dust_noise_node",
    parameters=[config]
    )

    ld.add_action(dust_noise_node)
    return ld