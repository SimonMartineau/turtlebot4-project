from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Declare a launch argument to choose between modified and default launch files
    use_mod_files = LaunchConfiguration('use_mod_files', default='false')

    return LaunchDescription([
        # Declare the launch argument
        DeclareLaunchArgument(
            'use_mod_files',
            default_value='false',
            description='Set to "true" to use modified launch files (view_robot_mod.launch.py, slam_mod.launch.py, all_noise.launch.py)'
        ),

        # Include view_robot_mod or view_robot based on the argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot4_viz'),
                '/launch/view_robot_mod.launch.py' if use_mod_files == 'true' else '/launch/view_robot.launch.py'
            ]),
        ),

        # Conditionally include all_noise based on the argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot4_bringup'), 
                '/launch/all_noise.launch.py'
            ]),
            condition=IfCondition(use_mod_files)
        ),

        # Conditionally include slam_mod or slam based on the argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot4_navigation'),
                '/launch/slam_mod.launch.py' if use_mod_files == 'true' else '/launch/slam.launch.py'
            ]),
        ),

        # Always include rqt_image_view
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
            output='screen'
        ),
    ])

