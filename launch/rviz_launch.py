import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_ultrasonic = get_package_share_directory('easywalk_ultrasonic')

    urdf_file  = os.path.join(pkg_ultrasonic, 'urdf', 'easywalk_ultrasonic.urdf.xacro')
    rviz_config = os.path.join(pkg_ultrasonic, 'config', 'walker.rviz')

    import xacro
    robot_desc = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
