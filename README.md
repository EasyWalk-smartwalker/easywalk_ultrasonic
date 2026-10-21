# easywalk_ultrasonic

ROS 2 package for the EasyWalk ring of six HC-SR04 ultrasonic sensors:
Arduino firmware plus a ROS 2 serial driver that publishes one calibrated
range per sensor.

Contents:

- `arduino/walker_sensors.ino`: Arduino Mega firmware (v2.0) that fires the
  6 HC-SR04 and streams the raw time of flight of every ping over serial.
- `easywalk_ultrasonic/serial_publisher.py`: reads the serial stream,
  converts it to calibrated distances and publishes them on ROS 2 topics.
- `config/sonar_calibration.yaml`: parameters of `serial_publisher`
  (temperature, per-sensor calibration, firmware configuration).
- `launch/sonar_launch.py`: starts `serial_publisher` with the calibration file.
- `config/hcsr04.yaml`, `urdf/`: sensor poses relative to `base_link` and the
  `hcsr04_sensors` URDF macro.
- Legacy, not used by the current pipeline: `easywalk_ultrasonic/decision_node.py`
  (threshold-based stop/warn/steer decisions on `walker/action`, no
  subscriber), `easywalk_ultrasonic/distance_printer.py` (prints the distances),
  `config/sensor_params.yaml` (thresholds of `decision_node`).

## Sensors

| Index | Name | Mounting                    | Notes                               |
|-------|------|-----------------------------|-------------------------------------|
| 0     | L30  | left side, 30° from front   | front-left                          |
| 1     | L15  | left side, 15° from front   | front-left                          |
| 2     | CTR  | front center                | forward-facing                      |
| 3     | R15  | right side, 15° from front  | front-right                         |
| 4     | R30  | right side, 30° from front  | front-right                         |
| 5     | BOT  | bottom center               | downward-facing, drop detection     |

Frame names: `<NAME>_hcsr04_link` (uppercase), e.g. `CTR_hcsr04_link`.

## Firmware output

`arduino/walker_sensors.ino` (115200 baud) does not convert to distance. For
every ping it sends

    R,<cycle>,<sensor>,<t_trig_us>,<tof_us>,<status>

with the sensor index above, the Arduino `micros()` at the trigger, the raw
round-trip time of flight in microseconds (0 if no echo) and `status`
(0 = echo, 1 = no echo before the timeout). Each cycle ends with
`C,<cycle>,<t_us>`. Lines starting with `#` are informational.

Mode (ring / single sensor), firing order, pauses, echo timeout and minimum
period per sensor can be changed over serial (see the sketch header);
`serial_publisher` sends the values of `config/sonar_calibration.yaml` at
startup. Lines in the old firmware format (`L30:12.3,...`) are still accepted,
with a warning.

## serial_publisher

Converts the time of flight with a temperature-dependent speed of sound,
`c = 331.3 + 0.606 T` m/s, applies the per-sensor correction
`d = gain * d + offset_m`, stamps each reading with the Arduino time of the
ping, and publishes:

- `walker/range/<name>` (`sensor_msgs/Range`, meters) following REP 117:
  `+inf` = no echo or beyond `max_range`, `-inf` = closer than `min_range`,
  `NaN` = invalid reading.
- `walker/distance/<name>` (`std_msgs/Float32`, centimeters): legacy topic for
  `decision_node` / `distance_printer`; `-1` = no echo, readings closer than
  `min_range` are published as `min_range`.
- `walker/sonar/raw` (`std_msgs/String`): raw serial lines; record it in every
  rosbag so that distances can be recomputed offline.

## Usage

```bash
ros2 launch easywalk_ultrasonic sonar_launch.py
ros2 launch easywalk_ultrasonic sonar_launch.py serial_port:=/dev/ttyACM1 temperature_c:=22.5
ros2 topic echo /walker/range/ctr
```

Calibration procedure: `paper/protocol/protocollo_sperimentale.tex`
(section "Calibrazione dei sensori a ultrasuoni") and the short checklist
`paper/protocol/primi_passi_sonar.pdf`.
