#!/usr/bin/env python3
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Key mappings: normalized left/right "effort" factors
# W = forward, S = backward, A = turn left, D = turn right, X = stop
MOVE_BINDINGS = {
    'w': (1.0, 1.0),
    's': (-1.0, -1.0),
    'a': (-0.5, 0.5),
    'd': (0.5, -0.5),
    'x': (0.0, 0.0),
}

def get_key(settings):
    """Non-blocking key capture from stdin."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    node = Node('teleop_keyboard_node')

    # Publish Twist to cmd_vel
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    # Speed here is treated as a linear-speed scale (m/s-ish)
    speed_param = node.declare_parameter('speed', 1.0)
    speed = float(speed_param.value)

    # Distance between left and right propulsion points (meters)
    # Adjust to match your boat's geometry
    base_width_param = node.declare_parameter('base_width', 0.25)
    base_width = float(base_width_param.value)

    settings = termios.tcgetattr(sys.stdin)

    print("Control the boat with WASD keys (publishing Twist to /cmd_vel):")
    print("  W: Forward")
    print("  S: Backward")
    print("  A: Turn left")
    print("  D: Turn right")
    print("  X: Stop")
    print("  Ctrl-C to quit.")

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key in MOVE_BINDINGS:
                left_cmd, right_cmd = MOVE_BINDINGS[key]

                # Scale left/right commands
                left_output = left_cmd * speed
                right_output = right_cmd * speed

                # Convert to diff-drive Twist
                linear_x = (left_output + right_output) / 2.0

                # Protect against divide-by-zero
                if abs(base_width) < 1e-6:
                    angular_z = 0.0
                else:
                    angular_z = (right_output - left_output) / base_width

                msg = Twist()
                msg.linear.x = float(linear_x)
                msg.angular.z = float(angular_z)

                pub.publish(msg)

                node.get_logger().info(
                    f"Key: {key} | left={left_output:.2f}, right={right_output:.2f} "
                    f"=> linear.x={linear_x:.2f}, angular.z={angular_z:.2f}"
                )

            elif key == '\x03':  # Ctrl-C
                break

    finally:
        # Publish stop
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        pub.publish(stop_msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
