import launch


def generate_launch_description():
    bag_recording = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            'robot_description',
            'scan_modified',
            'map',
            'waypoints',
            'tf',
            'tf_static'
        ],
    output='screen'
    )

    return launch.LaunchDescription([
        bag_recording
    ])

if __name__ == '__main__':
    generate_launch_description()