#!/usr/bin/env python3
import sys
import select
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Key mappings: normalized left/right "effort" factors
# W = forward, S = backward, A = turn left, D = turn right, X = stop
MOVE_BINDINGS = {
    'w': (1.0, 1.0),
    's': (0.0, 0.0),    # Disabled reverse (acts as stop)
    'a': (0.0, 1.0),    # Pivot left: Left motor stops, Right motor pushes
    'd': (1.0, 0.0),    # Pivot right: Left motor pushes, Right motor stops
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

    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    speed_param = node.declare_parameter('speed', 1.0)
    speed = float(speed_param.value)

    base_width_param = node.declare_parameter('base_width', 0.25)
    base_width = float(base_width_param.value)

    loop_rate_param = node.declare_parameter('repeat_rate', 20.0)
    repeat_rate = max(float(loop_rate_param.value), 1.0)
    repeat_period = 1.0 / repeat_rate

    settings = termios.tcgetattr(sys.stdin)

    print("Control the boat with WASD keys (publishing Twist to /cmd_vel):")
    print("  W: Forward")
    print("  S: Backward")
    print("  A: Turn left")
    print("  D: Turn right")
    print("  X: Stop")
    print("  Ctrl-C to quit.")

    last_msg = Twist()
    last_key = None
    next_publish_time = time.monotonic()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key(settings).lower()

            if key in MOVE_BINDINGS:
                left_cmd, right_cmd = MOVE_BINDINGS[key]

                left_output = left_cmd * speed
                right_output = right_cmd * speed

                linear_x = (left_output + right_output) / 2.0
                if abs(base_width) < 1e-6:
                    angular_z = 0.0
                else:
                    angular_z = (right_output - left_output) / base_width

                last_msg = Twist()
                last_msg.linear.x = float(linear_x)
                last_msg.angular.z = float(angular_z)

                pub.publish(last_msg)
                last_key = key if key != 'x' else None
                next_publish_time = time.monotonic() + repeat_period

                node.get_logger().info(
                    f"Key: {key} | left={left_output:.2f}, right={right_output:.2f} "
                    f"=> linear.x={linear_x:.2f}, angular.z={angular_z:.2f}"
                )

            elif key == '\x03':
                break
            else:
                now = time.monotonic()
                if last_key and now >= next_publish_time:
                    pub.publish(last_msg)
                    next_publish_time = now + repeat_period

            time.sleep(0.01)

    finally:
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        pub.publish(stop_msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
