import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_speed = float(self.declare_parameter('linear_speed', 0.3).value)
        self.angular_speed = float(self.declare_parameter('angular_speed', 0.5).value)
        #self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.sub_cmd_vel = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
    
    
    def cmd_vel_callback(self, msg: Twist):
        """Stores the latest command and updates the timestamp."""
        self.target_v = msg.linear.x
        self.target_w = msg.angular.z
        self.last_cmd_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            f'Publishing cmd_vel: linear.x={self.target_v} angular.z={self.target_w}'
        )
    
    '''
    def timer_callback(self):
        """Publish a constant Twist command for testing."""
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.pub.publish(msg)
        self.get_logger().info(
            #f'Publishing cmd_vel: linear.x={msg.linear.x:.2f} angular.z={msg.angular.z:.2f}'
        )
       ''' 

def main():
    rclpy.init()
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
