from setuptools import find_packages, setup

package_name = 'easywalk_ultrasonic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	('share/' + package_name + '/launch', [
    'launch/walker_launch.py',
    'launch/rviz_launch.py',
]),
        ('share/' + package_name + '/config', [
            'config/sensor_params.yaml',
            'config/walker.rviz',
            'config/hcsr04.yaml',
        ]),
        ('share/' + package_name + '/urdf', [
            'urdf/walker_sensors.urdf.xacro',
            'urdf/easywalk_ultrasonic.urdf.xacro',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='izzog',
    maintainer_email='izzo.giada23@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'serial_publisher = easywalk_ultrasonic.serial_publisher:main',
            'decision_node = easywalk_ultrasonic.decision_node:main',
	    'distance_printer = easywalk_ultrasonic.distance_printer:main',
        ],
    },
)
