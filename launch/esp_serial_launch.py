import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Load odometry calibration parameters from YAML
    odom_config = os.path.join(
        get_package_share_directory('esp_serial_v2_cpp'),
        'config', 'odometry_calibration.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/esp32_serial',
            description='Serial port for ESP32'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate'
        ),
        DeclareLaunchArgument(
            'max_rpm',
            default_value='115',
            description='Maximum motor RPM'
        ),
        DeclareLaunchArgument(
            'update_rate',
            default_value='100.0',
            description='Serial read rate in Hz'
        ),
        DeclareLaunchArgument(
            'cmd_vel_timeout',
            default_value='0.5',
            description='Timeout for cmd_vel in seconds'
        ),
        DeclareLaunchArgument(
            'invert_motor_l',
            default_value='true',
            description='Invert left motor rotation'
        ),
        DeclareLaunchArgument(
            'invert_motor_r',
            default_value='true',
            description='Invert right motor rotation'
        ),

        Node(
            package='esp_serial_v2_cpp',
            executable='esp_serial_ros2',
            name='esp_serial_ros2',
            output='screen',
            parameters=[odom_config, {
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'max_rpm': LaunchConfiguration('max_rpm'),
                'update_rate': LaunchConfiguration('update_rate'),
                'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout'),
                'invert_motor_l': LaunchConfiguration('invert_motor_l'),
                'invert_motor_r': LaunchConfiguration('invert_motor_r'),
            }]
        )
    ])
