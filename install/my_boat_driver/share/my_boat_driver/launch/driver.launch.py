import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # 1. Declare Launch Arguments (Configuration Variables)
    # These match the self.declare_parameter() calls in your Python code.
    serial_port_arg = DeclareLaunchArgument(
        'port', 
        default_value='/dev/ttyACM0',
        description='Serial port for the microcontroller'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate', 
        default_value='115200',
        description='Baud rate for serial communication'
    )

    base_width_arg = DeclareLaunchArgument(
        'base_width', 
        default_value='0.25',#meters
        description='Distance between motors (meters)'
    )

    prop_radius_arg = DeclareLaunchArgument(
        'prop_radius', 
        default_value='0.03',#meters
        description='Radius of the propeller (meters)'
    )

    max_rpm_arg = DeclareLaunchArgument(
        'max_rpm', 
        default_value='60.0',
        description='Maximum RPM limit for the motors'
    )

    cmd_timeout_arg = DeclareLaunchArgument(
        'cmd_timeout', 
        default_value='2.0',
        description='Time in seconds before stopping if no command is received'
    )

    # 2. Define the Node
    motor_driver_node = Node(
        package='my_boat_driver',          # package name
        executable='motor_driver_node',    # remember the setup.py entry point
        name='boat_base_controller',       # Name of the node in the ROS graph
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'base_width': LaunchConfiguration('base_width'),
            'prop_radius': LaunchConfiguration('prop_radius'),
            'max_rpm': LaunchConfiguration('max_rpm'),
            'cmd_timeout': LaunchConfiguration('cmd_timeout')
        }]
    )

    # 3. Return the LaunchDescription
    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        base_width_arg,
        prop_radius_arg,
        max_rpm_arg,
        cmd_timeout_arg,
        motor_driver_node
    ])