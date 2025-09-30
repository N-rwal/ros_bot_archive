from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch LiDAR node (from another package)
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 1000000
            }],
            #output='screen'  # Optional: Show logs in terminal
        ),
        
        # Launch your custom UDP sender node
        Node(
            package='my_package',
            executable='udp_sender_node',
            #output='screen'  # Print logs to terminal
        ),
    ])
