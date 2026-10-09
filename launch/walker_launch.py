import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg = get_package_share_directory('easywalk_ultrasonic')
    params = os.path.join(pkg, 'config', 'sensor_params.yaml')

    return LaunchDescription([
        Node(
            package='easywalk_ultrasonic',
            executable='serial_publisher',
            name='serial_publisher',
            output='screen',
        ),
        Node(
            package='easywalk_ultrasonic',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='easywalk_ultrasonic',
            executable='distance_printer',
            name='distance_printer',
            output='screen',
        ),
    ])
