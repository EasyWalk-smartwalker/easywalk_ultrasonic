# Avvia il driver dei sonar (serial_publisher) con il file di calibrazione.
#
#   ros2 launch easywalk_ultrasonic sonar_launch.py
#   ros2 launch easywalk_ultrasonic sonar_launch.py serial_port:=/dev/ttyACM1 temperature_c:=22.5

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    calib = os.path.join(get_package_share_directory('easywalk_ultrasonic'),
                         'config', 'sonar_calibration.yaml')

    serial_port = LaunchConfiguration('serial_port')
    temperature = LaunchConfiguration('temperature_c')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('temperature_c', default_value='20.0',
                              description='Temperatura della sessione [C]'),
        Node(
            package='easywalk_ultrasonic',
            executable='serial_publisher',
            name='serial_publisher',
            output='screen',
            # I valori da riga di comando sovrascrivono quelli del file.
            parameters=[calib, {
                'serial_port': serial_port,
                # forza il tipo: "temperature_c:=20" verrebbe letto come intero
                'temperature_c': ParameterValue(temperature, value_type=float),
            }],
        ),
    ])
