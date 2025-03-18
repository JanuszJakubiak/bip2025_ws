import rclpy
from rclpy.node import Node
from geometry_msgs.msg import AccelStamped
import time
import random

class Publisher(Node):
    def __init__(self):
        super().__init__('acc_publisher')
        self.publisher_= self.create_publisher(AccelStamped, 'acc_simulation', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.msg_=AccelStamped()
        self.msg_.accel.linear.z=-1.0
        self.msg_.header.frame_id='map'
    
    def timer_callback(self):        
        dx=random.uniform(-0.05,0.05)
        dy=random.uniform(-0.05,0.05)
        dz=random.uniform(-0.1,0.1)
        self.msg_.accel.linear.x+=dx
        self.msg_.accel.linear.y+=dy
        self.msg_.accel.linear.z+=dz

        self.msg_.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.msg_)

        
def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()

    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
