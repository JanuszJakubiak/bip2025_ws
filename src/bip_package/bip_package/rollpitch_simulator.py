import rclpy
from rclpy.node import Node
from geometry_msgs.msg import AccelStamped
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
import time
import random
import math
import numpy as np


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class Publisher(Node):
    def __init__(self):
        super().__init__('acc_publisher')
        self.publisher_= self.create_publisher(AccelStamped, 'acc_simulation', 10)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.tf_broadcaster = TransformBroadcaster(self)            
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.msg_=AccelStamped()
        self.msg_.accel.linear.z=-1.0
        self.msg_.header.frame_id='world'

        marker = Marker()
        marker.header.frame_id = "board"  # Reference the board frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cylinder"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Set Position and orientation to "0" (relatively to the "board" frame)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = -0.05
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set Color
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        # Set size
        marker.scale.x=4.0
        marker.scale.y=4.0
        marker.scale.z=0.1
        self.marker_=marker



    def timer_callback(self):        
        dx=random.uniform(-0.05,0.05)
        dy=random.uniform(-0.05,0.05)
        dz=random.uniform(-0.1,0.1)
        self.msg_.accel.linear.x+=dx
        self.msg_.accel.linear.y+=dy
        self.msg_.accel.linear.z+=dz

        self.msg_.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.msg_)

        #Publish marker
        self.marker_.header.stamp = self.get_clock().now().to_msg()
        self.marker_publisher.publish(self.marker_)  

        #Publish transformation frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'board'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0+dx, 0+dz, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


        
def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()

    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
