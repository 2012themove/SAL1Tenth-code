import rclpy
import numpy as np
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from sensor_msgs.msg import Imu
from gps_nav_interfaces.msg import Position
from geometry_msgs.msg import PoseStamped, TransformStamped

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('pose_creator_vehicle')

        #UNUSED
        #self.subscription1 = self.create_subscription(Imu,'/sensors/imu/raw',self.rawimu_callback,10)
        #self.subscription1  # prevent unused variable warning

        #Vicon Subscriptions
        self.car1ViconSub = self.create_subscription(Position,'/vicon/car1/car1',self.car1_vicon_callback,10)
        self.car1ViconSub  # prevent unused variable warning
        self.car2ViconSub = self.create_subscription(Position,'/vicon/car2/car2',self.car2_vicon_callback,10)
        self.car2ViconSub  # prevent unused variable warning

        #Pose and tf Publishers/brodcasters
        self.car1PosePub = self.create_publisher(PoseStamped, '/car1/pos', 10)
        self.car2PosePub = self.create_publisher(PoseStamped, '/car2/pos', 10)
        #self.car1TfPub = self.create_publisher(TransformStamped, 'tf/car1', 10)
        #self.car2TfPub = self.create_publisher(TransformStamped, 'tf/car2', 10)  
        self.br = TransformBroadcaster(self)
        
        #Vehicle 1 Info
        self.car1_quaternion = np.array([0.0, 0.0, 0.0, 0.0]) #x,y,z,w
        self.car1_pos = np.array([0.0, 0.0]) # x,y

        #Vehicle 2 Info
        self.car2_quaternion = np.array([0.0, 0.0, 0.0, 0.0]) #x,y,z,w
        self.car2_pos = np.array([0.0, 0.0]) # x,y

        #Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
    

    # Publish poses and transformation
    def timer_callback(self):
        car1_pos_msg = PoseStamped()
        car1_pos_msg.header.frame_id = "vehicle_viz"
        car1_pos_msg.pose.position.x = self.car1_pos[0]
        car1_pos_msg.pose.position.y = self.car1_pos[1]
        car1_pos_msg.pose.position.z = 0.0
        car1_pos_msg.pose.orientation.x = self.car1_quaternion[0]
        car1_pos_msg.pose.orientation.y = self.car1_quaternion[1]
        car1_pos_msg.pose.orientation.z = self.car1_quaternion[2]
        car1_pos_msg.pose.orientation.w = self.car1_quaternion[3]

        car1_tf_msg = TransformStamped()
        car1_tf_msg.header.frame_id = "vehicle_viz"
        car1_tf_msg.child_frame_id = "car1"
        car1_tf_msg.transform.translation.x = self.car1_pos[0]
        car1_tf_msg.transform.translation.y = self.car1_pos[1]
        car1_tf_msg.transform.translation.z = 0.0
        car1_tf_msg.transform.rotation.x = self.car1_quaternion[0]
        car1_tf_msg.transform.rotation.y = self.car1_quaternion[1]
        car1_tf_msg.transform.rotation.z = self.car1_quaternion[2]
        car1_tf_msg.transform.rotation.w = self.car1_quaternion[3]

        car2_pos_msg = PoseStamped()
        car2_pos_msg.header.frame_id = "vehicle_viz"
        car2_pos_msg.pose.position.x = self.car2_pos[0]
        car2_pos_msg.pose.position.y = self.car2_pos[1]
        car2_pos_msg.pose.position.z = 0.0
        car2_pos_msg.pose.orientation.x = self.car2_quaternion[0]
        car2_pos_msg.pose.orientation.y = self.car2_quaternion[1]
        car2_pos_msg.pose.orientation.z = self.car2_quaternion[2]
        car2_pos_msg.pose.orientation.w = self.car2_quaternion[3]

        car2_tf_msg = TransformStamped()
        car2_tf_msg.header.frame_id = "vehicle_viz"
        car2_tf_msg.child_frame_id = "car2"
        car2_tf_msg.transform.translation.x = self.car2_pos[0]
        car2_tf_msg.transform.translation.y = self.car2_pos[1]
        car2_tf_msg.transform.translation.z = 0.0
        car2_tf_msg.transform.rotation.x = self.car2_quaternion[0]
        car2_tf_msg.transform.rotation.y = self.car2_quaternion[1]
        car2_tf_msg.transform.rotation.z = self.car2_quaternion[2]
        car2_tf_msg.transform.rotation.w = self.car2_quaternion[3]


        self.car1PosePub.publish(car1_pos_msg)
        self.car2PosePub.publish(car2_pos_msg)
        self.br.sendTransform(car1_tf_msg)
        self.br.sendTransform(car2_tf_msg)
        #self.car1TfPub.publish(car1_tf_msg)
        #self.car2TfPub.publish(car2_tf_msg)
        
    # Read car1 Pose
    def car1_vicon_callback(self, msg):
        self.car1_pos[0] = msg.x_trans/1000.0
        self.car1_pos[1] = msg.y_trans/1000.0
        self.car1_quaternion[0] = msg.x_rot
        self.car1_quaternion[1] = msg.y_rot
        self.car1_quaternion[2] = msg.z_rot
        self.car1_quaternion[3] = msg.w
        
    # Read car2 Pose
    def car2_vicon_callback(self, msg):
        self.car2_pos[0] = msg.x_trans/1000.0
        self.car2_pos[1] = msg.y_trans/1000.0
        self.car2_quaternion[0] = msg.x_rot
        self.car2_quaternion[1] = msg.y_rot
        self.car2_quaternion[2] = msg.z_rot
        self.car2_quaternion[3] = msg.w

    ##UNUSUED
    def rawimu_callback(self, msg):
        # self.car1_quaternion[0] = msg.orientation.x
        # self.car1_quaternion[1] = msg.orientation.y
        # self.car1_quaternion[2] = msg.orientation.z
        # self.car1_quaternion[3] = msg.orientation.w
        x = 6


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