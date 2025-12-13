#!/usr/bin/env python3
import math
import serial
import serial.serialutil

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class BoatBaseController(Node):
    def __init__(self):
        super().__init__('boat_base_controller')

        # ---------- Parameters ----------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('base_width', 0.25)      # distance between motors (m)
        self.declare_parameter('prop_radius', 0.03)     # propeller radius (m)
        self.declare_parameter('max_rpm', 60.0)         # <= your motor limit
        self.declare_parameter('cmd_timeout', 0.3)      # seconds, stop if no cmd_vel
        
        self.serial_port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.base_width = float(self.get_parameter('base_width').value)
        self.prop_radius = float(self.get_parameter('prop_radius').value)
        self.max_rpm = float(self.get_parameter('max_rpm').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
    
        
        # open the serial monitor 
        self.ser = None
        try:
            self.ser = serial.Serial(
                self.serial_port,
                self.baudrate,
                timeout=0.05,
                write_timeout=1.0
            )
            self.get_logger().info(f"Opened serial port {self.serial_port} @ {self.baudrate}")
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.serial_port}: {e}")

        #State 
        self.current_v = 0.0      # m/s
        self.current_w = 0.0      # rad/s
        self.last_cmd_time = self.get_clock().now()

        #ROS Interfaces 
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 20 Hz control loop
        self.frequency_ = 0.1
        self.timer = self.create_timer(self.frequency_, self.update)

    # Callbacks

    def cmd_vel_callback(self, msg: Twist):
        """Handles incoming cmd_vel messages."""
        self.current_v = msg.linear.x
        self.current_w = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def update(self):
        """Called periodically. Computes RPM and sends to Arduino."""
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9

        if dt > self.cmd_timeout:
            # No command recently → stop the boat
            v = 0.0
            w = 0.0
        else:
            v = self.current_v
            w = self.current_w

        left_rpm, right_rpm = self.twist_to_rpm(v, w)
        self.send_to_arduino(left_rpm, right_rpm)

    # Helpers

    def twist_to_rpm(self, v: float, w: float):
        """
        Convert (v, w) to left/right RPM using a simple differential model.
        v: linear velocity (m/s)
        w: angular velocity (rad/s)
        """
        B = self.base_width
        r = self.prop_radius

        # Linear speed of each prop (m/s)
        v_left = v - (w * B / 2.0)
        v_right = v + (w * B / 2.0)

        # Convert linear speed (m/s) -> shaft angular speed (rad/s)
        # v = r * omega  =>  omega = v / r
        if r <= 0:
            self.get_logger().warn("prop_radius <= 0, defaulting RPM to 0")
            return 0.0, 0.0

        omega_left = v_left / r
        omega_right = v_right / r

        # rad/s -> RPM   (omega * 60 / (2π))
        rpm_left = omega_left * 60.0 / (2.0 * math.pi)
        rpm_right = omega_right * 60.0 / (2.0 * math.pi)

        # Clamp to max RPM (±60 by default)
        rpm_left = max(min(rpm_left, self.max_rpm), -self.max_rpm)
        rpm_right = max(min(rpm_right, self.max_rpm), -self.max_rpm)
        

        return rpm_left, rpm_right

    def send_to_arduino(self, left_rpm: float, right_rpm: float):
        """
        Format and write the command over serial.

        Protocol (ASCII line):
            CMD <left_rpm> <right_rpm>\n

        Examples:
            CMD 30.0 30.0
            CMD -15.5 20.0
        """
        if self.ser is None or not self.ser.is_open:
            return

        cmd = f"CMD {left_rpm:.1f}  {right_rpm:.1f}\n"
        self.get_logger().info(f"Serial TX -> {cmd.strip()}")

        try:
            self.ser.reset_input_buffer()
            self.ser.write(cmd.encode('ascii'))
            response = self.ser.readline().decode('ascii').strip()
            if response:
                self.get_logger().info(f"Serial RX from arduino <- {response}")
        except serial.SerialTimeoutException:
            self.get_logger().warn("Serial write timed out! Arduino is too slow or disconnected.")
            # Optional: flush buffers to clear the blockage
            self.ser.reset_output_buffer()
            
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    # ===== Cleanup =====

    def destroy_node(self):
        # Try to stop motors on shutdown
        self.send_to_arduino(0.0, 0.0)
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BoatBaseController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()