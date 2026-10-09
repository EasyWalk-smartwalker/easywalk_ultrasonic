# easywalk_ultrasonic

ROS 2 package for the EasyWalk HC-SR04 ultrasonic sensor integration,
split out of `easywalk_description` to keep that package focused on robot
geometry only.

Contents:

- `arduino/walker_sensors.ino`: Arduino sketch reading the 6 HC-SR04 sensors
  and streaming distances over serial.
- `easywalk_ultrasonic/serial_publisher.py`: reads the Arduino serial stream
  and republishes it on ROS 2 topics.
- `easywalk_ultrasonic/decision_node.py`: turns sensor distances into
  stop/warn/steer decisions.
- `easywalk_ultrasonic/distance_printer.py`: debug node that prints the
  published distances.
- `urdf/walker_sensors.urdf.xacro`: the `hcsr04_sensors` macro modeling the
  6 HC-SR04 ultrasonic distance sensors (L30, L15, CTR, R15, R30, BOT) as
  fixed joints.
- `urdf/easywalk_ultrasonic.urdf.xacro`: combined entry point that includes
  the base `easywalk_description` robot and extends it with the sensor
  links/joints above.
- `config/hcsr04.yaml`: sensor positions, relative to `base_link`.
- `config/sensor_params.yaml`: thresholds and parameters for `decision_node`.
- `launch/walker_launch.py`: runs the sensor driver + decision + debug nodes.
- `launch/rviz_launch.py`: launches `robot_state_publisher` +
  `joint_state_publisher` + RViz with the combined URDF (base robot +
  sensors) and detection cones.

## HC-SR04 ultrasonic sensors

Six HC-SR04 (5V version) ultrasonic distance sensors are modeled as fixed
joints off `base_link`, following the same yaml-driven macro pattern used
by `easywalk_description` (see `urdf/walker_sensors.urdf.xacro` and
`config/hcsr04.yaml`).

Sensor layout (position measured from the center of the top plate):

| Name | Position               | Notes                     |
|------|-------------------------|----------------------------|
| L30  | left side, 30° from front  | front-left angled sensor |
| L15  | left side, 15° from front  | front-left angled sensor |
| CTR  | front center             | forward-facing            |
| R15  | right side, 15° from front | front-right angled sensor |
| R30  | right side, 30° from front | front-right angled sensor |
| BOT  | bottom center             | downward-facing, drop/edge detection |

Link/frame names: `<NAME>_hcsr04_link` (uppercase), e.g. `L30_hcsr04_link`,
`CTR_hcsr04_link`, `BOT_hcsr04_link`.

This package reads the sensors over serial and publishes:
- `walker/distance/<name>` (`std_msgs/Float32`, centimeters) — used by `decision_node` and `distance_printer`.
- `walker/range/<name>` (`sensor_msgs/Range`, meters) — used for RViz visualization (`frame_id` matches the link names above).

Run `ros2 launch easywalk_ultrasonic rviz_launch.py` to view the full robot
with sensor detection cones in RViz, and `ros2 launch easywalk_ultrasonic
walker_launch.py` to run the live sensor pipeline on the robot.
