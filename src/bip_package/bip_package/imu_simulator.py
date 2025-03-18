import rclpy
from rclpy.node import Node
from geometry_msgs.msg import AccelStamped
import time
import random
from rcl_interfaces.msg import ParameterDescriptor

class Publisher(Node):
    def __init__(self):
        super().__init__('acc_publisher')
        self.publisher_= self.create_publisher(AccelStamped, 'acc_simulation', 10)
        scale_descriptor = ParameterDescriptor(description='Scaling coefficient')
        self.declare_parameter('scale', 1.0, scale_descriptor)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.msg_=AccelStamped()
        self.msg_.accel.linear.z=-1.0
        self.msg_.header.frame_id='map'
    
    def timer_callback(self):
        scale_param = self.get_parameter('scale').get_parameter_value().double_value
        dx=random.uniform(-0.05*scale_param,0.05*scale_param)
        dy=random.uniform(-0.05*scale_param,0.05*scale_param)
        dz=random.uniform(-0.1*scale_param,0.1*scale_param)
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
