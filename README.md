# ESP32 Serial Parser (ROS2 C++)

ROS2 C++ node for serial communication with ESP32 microcontroller.

## Features

- Serial port communication (configurable port & baud rate)
- JSON message parsing with nlohmann/json
- `cmd_vel` subscription for velocity commands
- Sensor data publishing (encoders, IMU, battery)
- Diagnostic updater for health monitoring
- Configurable motor parameters (wheel radius, separation, max RPM)

## Dependencies

- `rclcpp`
- `serial`
- `nlohmann_json`
- `std_msgs`
- `geometry_msgs`
- `std_srvs`
- `diagnostic_updater`
- `nav_msgs`
- `tf2`
- `tf2_ros`

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/esp32_serial` | Serial port device |
| `baud_rate` | `115200` | Baud rate |
| `wheel_radius` | `0.0473` | Wheel radius (m) |
| `wheel_separation` | `0.1796` | Wheel tread (m) |
| `max_rpm` | `115` | Maximum motor RPM |
| `update_rate` | `100.0` | Serial read rate (Hz) |
| `cmd_vel_timeout` | `0.5` | cmd_vel timeout (s) |
| `invert_motor_l` | `false` | Invert left motor |
| `invert_motor_r` | `false` | Invert right motor |

## Usage

### esp_serial_ros2 (Serial Communication Node)

```bash
ros2 launch esp_serial_v2_cpp esp_serial_launch.py
```

With custom parameters:

```bash
ros2 launch esp_serial_v2_cpp esp_serial_launch.py serial_port:=/dev/ttyUSB0 baud_rate:=921600
```

### odometry_publisher (Odometry & TF Publisher)

```bash
ros2 run esp_serial_v2_cpp odometry_publisher
```

With custom parameters:

```bash
ros2 run esp_serial_v2_cpp odometry_publisher --ros-args -p wheel_radius:=0.0473 -p wheel_separation:=0.1796 -p publish_rate:=50.0
```

## Topics

### Subscribed
- `/cmd_vel` (`geometry_msgs/Twist`) - Velocity commands

### Published (esp_serial_ros2)
- `/esp/speed_l` (`std_msgs/Int16`) - Left motor speed
- `/esp/speed_r` (`std_msgs/Int16`) - Right motor speed
- `/esp/position_l` (`std_msgs/Int16`) - Left encoder position
- `/esp/position_r` (`std_msgs/Int16`) - Right encoder position
- `/esp/position_l_rad` (`std_msgs/Float64`) - Left wheel position (rad)
- `/esp/position_r_rad` (`std_msgs/Float64`) - Right wheel position (rad)
- `/esp/battery_voltage` (`std_msgs/Float32`) - Battery voltage
- `/esp/timestamp` (`std_msgs/UInt32`) - ESP32 timestamp
- `/diagnostics` (`diagnostic_msgs/DiagnosticArray`) - Diagnostics

### Subscribed (odometry_publisher)
- `/esp/position_l_rad` (`std_msgs/Float64`) - Left wheel position (rad)
- `/esp/position_r_rad` (`std_msgs/Float64`) - Right wheel position (rad)

### Published (odometry_publisher)
- `/odom` (`nav_msgs/Odometry`) - Odometry
- `/tf` (`tf2_msgs/TFMessage`) - Odom to base_link transform

## License

Apache License 2.0
