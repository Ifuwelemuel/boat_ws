#!/usr/bin/env python3
import sys
import select
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')
        
        # Publisher
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parameters
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('base_width', 0.25)
        
        self.speed = self.get_parameter('speed').value
        self.base_width = self.get_parameter('base_width').value
        
        # State Timers (Unix timestamps)
        self.last_linear_press = 0.0
        self.last_angular_press = 0.0
        self.key_timeout = 0.2  # Seconds to "remember" a key press
        
        # Current Target Commands
        self.target_linear = 0.0
        self.target_angular = 0.0

        # Terminal Settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info(
            "\n------------------------------------\n"
            "   COMBINED KEYBOARD TELEOP\n"
            "------------------------------------\n"
            "   W           : Forward\n"
            "   A / D       : Left / Right\n"
            "   W + A       : Forward Left Arc\n"
            "   W + D       : Forward Right Arc\n"
            "   S / X       : Stop (No Reverse)\n"
            "   CTRL-C      : Quit\n"
            "------------------------------------\n"
        )
        
        # Main Loop Timer (20 Hz)
        self.create_timer(0.05, self.update_loop)

    def get_key(self):
        """Reads a single key from stdin without blocking."""
        tty.setraw(sys.stdin.fileno())
        # Check if data is available (timeout=0 for instant check)
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        key = ''
        if rlist:
            key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def update_loop(self):
        """Checks for keys, updates state, and publishes Twist."""
        now = time.time()
        
        # 1. Drain input buffer to get ALL pressed keys
        # This handles the 'wawawa' stream when two keys are held
        while True:
            key = self.get_key()
            if key == '':
                break
            
            if key == '\x03':  # CTRL-C
                self.stop_and_exit()

            # Update State based on Key
            if key == 'w':
                self.target_linear = 1.0
                self.last_linear_press = now
            elif key == 's' or key == 'x':
                self.target_linear = 0.0
                # Force expiration of linear command
                self.last_linear_press = 0.0 
            
            elif key == 'a':
                self.target_angular = 1.0  # Turn Left
                self.last_angular_press = now
            elif key == 'd':
                self.target_angular = -1.0 # Turn Right
                self.last_angular_press = now

        # 2. Check for decay (if key released)
        # If we haven't seen 'w' in 0.2 seconds, stop moving forward
        if (now - self.last_linear_press) > self.key_timeout:
            self.target_linear = 0.0
            
        # If we haven't seen 'a' or 'd' in 0.2 seconds, stop turning
        if (now - self.last_angular_press) > self.key_timeout:
            self.target_angular = 0.0

        # 3. Calculate and Publish Twist
        msg = Twist()
        
        # Apply Speed Scalar
        msg.linear.x = self.target_linear * self.speed
        
        # Simple Logic: 
        # If Only W: Linear=1, Angular=0
        # If W + A:  Linear=1, Angular=1
        msg.angular.z = self.target_angular * self.speed

        self.pub.publish(msg)

    def stop_and_exit(self):
        stop_msg = Twist()
        self.pub.publish(stop_msg)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        print("\nQuitting...")
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.stop_and_exit()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()