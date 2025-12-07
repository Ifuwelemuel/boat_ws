import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.3    # forward speed (m/s)
        msg.angular.z = 0.5   # yaw rate (rad/s)
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing cmd_vel: linear.x={msg.linear.x} angular.z={msg.angular.z}')

def main():
    rclpy.init()
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
