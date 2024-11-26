import rclpy
import math
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped

import utm

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('pose_estimate_converter')
        self.subscription1 = self.create_subscription(PoseStamped,'vehicle_pose',self.listener_callback1,10)
        self.subscription1  # prevent unused variable warning
        self.gps_file = open('final_gps_data.txt','w')
        #self.angle_file = open('angle_data.txt','w')
        
        
        
    def listener_callback1(self, msg):
        latitude = msg.pose.position.y
        longitude = msg.pose.position.x
        self.gps_file.write(f'{latitude},{longitude}\n')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()