from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition



def generate_launch_description():
    # Declare a launch argument to choose between modified and default launch files
    use_mod_files = LaunchConfiguration('use_mod_files', default='false')

    return LaunchDescription([
        # Declare the launch argument
        DeclareLaunchArgument(
            'use_mod_files',
            default_value='false',
            description='Set to "true" to use modified launch files (view_rtabmap.launch.py, rtabmap_bringup_mod.launch.py, all_noise.launch.py)'
        ),

        DeclareLaunchArgument(
            'rtabmap_args',
            default_value='--delete_db_on_start',
            description='Arguments to pass to RTAB-Map'
        ),

        # Include view_robot_mod or view_robot based on the argument, with a 5-second delay for QoS reasons I don't know yet.
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        get_package_share_directory('turtlebot4_viz'),
                        '/launch/view_rtabmap_mod.launch.py' if use_mod_files == 'true' else '/launch/view_rtabmap.launch.py'
                    ]),
                ),
            ],
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
                get_package_share_directory('turtlebot4_bringup'),
                '/launch/rtabmap_bringup_mod.launch.py' if use_mod_files == 'true' else '/launch/rtabmap_bringup.launch.py'
            ]),
            #launch_arguments={'rtabmap_args': '--delete_db_on_start'}.items()
        ),

        # Always include rqt_image_view
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
            output='screen'
        ),
    ])
