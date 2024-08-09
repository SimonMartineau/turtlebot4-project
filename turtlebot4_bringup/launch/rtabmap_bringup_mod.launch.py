import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    # static transforms
    camera_robot_st = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=['--x', '0.188', '--y', '0', '--z', '0.36', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'pi3_link']
        )


    # realsense stuff
    realsense_launch_dir = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )
    print(realsense_launch_dir)

    realsense_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_dir),
        launch_arguments = {
            'camera_name': 'pi3',
            'depth_module.depth_profile': '480x270x5',
            'rgb_camera.color_profile': '480x270x5',
            'enable_sync': 'true'}.items()
        )

    realsense_with_namespace = GroupAction(
        actions = [
            PushRosNamespace('/visual_slam'),
            realsense_launcher])

    # rtabmap stuff
    rtabmap_launch_dir = os.path.join(
        get_package_share_directory('rtabmap_launch'),
        'launch',
        'rtabmap.launch.py'
    )
    print(rtabmap_launch_dir)


    rtabmap_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_dir),
        launch_arguments = {
            'rgb_topic': '/image_modified',
            'depth_topic': '/camera/pi3/depth/image_rect_raw',
            'camera_info_topic': '/camera/pi3/color/camera_info',
            'frame_id': 'base_link',
            'approx_sync': 'true',
            'wait_imu_to_init': 'false',
            'imu_topic': '/imu',
            'qos': '1',
            'qos_imu': '2',
            'rtabmap_viz': 'false',
            'rviz': 'false'}.items()
        )

    rtabmap_with_namespace = GroupAction(
        actions = [
            PushRosNamespace('/visual_slam'),
            rtabmap_launcher
    ])

    return LaunchDescription([
        camera_robot_st,
        #realsense_with_namespace,
        realsense_launcher,
        #rtabmap_with_namespace,
        rtabmap_launcher,
    ])