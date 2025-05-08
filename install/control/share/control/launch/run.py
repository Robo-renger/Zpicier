from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control',
            executable='imu_node',
            name='imu_sender_node',
            output='screen'
        ),
        Node(
            package='control',
            executable='depth_node',
            name='depth_sender_node',
            output='screen'
        ),
        Node(
            package='control',
            executable='joystick_channel_node',
            name='joystick_channel_node',
            output='screen'
        ),
        Node(
            package='control',
            executable='gui_streamer',
            name='gui_streamer',
            output='screen'
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}]

        ),
    ])
