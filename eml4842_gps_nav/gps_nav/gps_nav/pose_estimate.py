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
        super().__init__('pose_estimate')
        self.subscription1 = self.create_subscription(NavSatFix,'gps',self.listener_callback1,10)
        self.subscription1  # prevent unused variable warning
        self.subscription2 = self.create_subscription(Odometry,'odometry',self.listener_callback2,10)
        self.subscription2  # prevent unused variable warning
        self.gps_file = open('gps_data.txt','w')
        #self.angle_file = open('angle_data.txt','w')
        self.publisher = self.create_publisher(PoseStamped, 'vehicle_pose',10)
        self.main_timer = self.create_timer(timer_period_sec=0.1, callback=self.main_timer_callback)
        
        self.out_msg = PoseStamped()
        
        
        
    def listener_callback1(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        self.gps_file.write(f'{latitude},{longitude}\n')

        self.out_msg.header.stamp = self.get_clock().now().to_msg()
        # self.out_msg.pose.position.x = latitude
        # self.out_msg.pose.position.y = longitude #change from lat/long utm
        
        easting, northing, zone_number, zone_letter = utm.from_latlon(latitude, longitude)
        self.out_msg.pose.position.x = easting
        self.out_msg.pose.position.y = northing
        self.out_msg.pose.position.z = 0.0
        
    
    def listener_callback2(self,msg):
        orientation_x = msg.pose.pose.orientation.x
        orientation_y = msg.pose.pose.orientation.y
        orientation_z = msg.pose.pose.orientation.z
        orientation_w = msg.pose.pose.orientation.w

        angle = 2*math.atan2(orientation_z,orientation_w)
        #self.angle_file.write(f'X: {orientation_x}, Y: {orientation_y}, Z: {orientation_z}, W: {orientation_w}, angle: {angle}\n')
        
        self.out_msg.header.stamp = self.get_clock().now().to_msg()
        self.out_msg.pose.orientation.x = orientation_x
        self.out_msg.pose.orientation.y = orientation_y
        self.out_msg.pose.orientation.z = orientation_z
        self.out_msg.pose.orientation.w = orientation_w
    
    def main_timer_callback(self):
        # stupid shit to comment
        # easting, northing, zone_number, zone_letter = utm.from_latlon(29.647063, -82.346571)
        # self.out_msg.pose.position.x = 369668.6478483258
        # self.out_msg.pose.position.y = 3280420.299123118
        # self.out_msg.pose.position.z = 0.0

        self.publisher.publish(self.out_msg)


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