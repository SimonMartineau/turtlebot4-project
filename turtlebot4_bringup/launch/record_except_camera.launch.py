import launch


def generate_launch_description():
    bag_recording = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            'battery_state',
            'cliff_intensity',
            'cmd_audio',
            'cmd_lightring',
            'cmd_vel',
            'diagnostics',
            'diagnostics_agg',
            'diagnostics_toplevel_state',
            'dock_status',
            'function_calls',
            'hazard_detection',
            'hmi/buttons',
            'hmi/display',
            'hmi/display/message',
            'hmi/led',
            'imu',
            'interface_buttons',
            'ip',
            'ir_intensity',
            'ir_opcode',
            'joint_states',
            'joy',
            'joy/set_feedback',
            'kidnap_status',
            'mobility_monitor/transition_event',
            'mouse',
            'odom',
            'parameter_events',
            'robot_description',
            'robot_state/transition_event',
            'rosout',
            'scan',
            'slip_status',
            'static_transform/transition_event',
            'stereo/depth',
            'stop_status',
            'tf',
            'tf_static',
            'wheel_status',
            'wheel_ticks',
            'wheel_vels',
        ],
    output='screen'
    )

    return launch.LaunchDescription([
        bag_recording
    ])

if __name__ == '__main__':
    generate_launch_description()