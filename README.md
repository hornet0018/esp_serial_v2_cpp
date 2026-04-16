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

```bash
ros2 launch esp_serial_v2_cpp esp_serial_launch.py
```

With custom parameters:

```bash
ros2 launch esp_serial_v2_cpp esp_serial_launch.py serial_port:=/dev/ttyUSB0 baud_rate:=921600
```

## Topics

### Subscribed
- `/cmd_vel` (`geometry_msgs/Twist`) - Velocity commands

### Published
- `/encoder/l` (`std_msgs/Int16`) - Left encoder count
- `/encoder/r` (`std_msgs/Int16`) - Right encoder count
- `/imu/...` - IMU data
- `/battery/voltage` (`std_msgs/Float32`) - Battery voltage
- `/diagnostics` (`diagnostic_msgs/DiagnosticArray`) - Diagnostics

## License

Apache License 2.0
