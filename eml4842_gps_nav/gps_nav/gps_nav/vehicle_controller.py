import math
import numpy as np
import csv

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

from gps_nav_interfaces.msg import CurrentGoalPose

from gps_nav.uf_support.route_support import determine_arc_radius
from gps_nav.uf_support.route_support import get_rad_of_curvature_to_carrot
from gps_nav.uf_support.route_support import get_cross_track_and_heading_error

class VehicleController(Node):
    def __init__(self):
        super().__init__("vehicle_controller")

        self.declare_parameter("L_wheelbase_m", 0.33)

        self.subscription_pos_car1 = self.create_subscription(PoseStamped, "/car1/pos", self.vehicle_pose_callback_car1, 1)
        self.subscription_goal_car1 = self.create_subscription(CurrentGoalPose, "car1/current_goal_pose", self.current_goal_pose_callback_car1, 1)

        self.subscription_pos_car2 = self.create_subscription(PoseStamped, "/car2/pos", self.vehicle_pose_callback_car2, 1)
        self.subscription_goal_car2 = self.create_subscription(CurrentGoalPose, "car2/current_goal_pose", self.current_goal_pose_callback_car2, 1)

        self.publisher_car1 = self.create_publisher(AckermannDriveStamped, "ackermann_cmd", 10)
        self.publisher_car2 = self.create_publisher(AckermannDriveStamped, "car2/ackermann_cmd", 10)

        self.subscription_estop = self.create_subscription(Int8, "e_stop", self.e_stop_callback, 10)

        # set up the timer (0.1 sec) to send over the current_carrot message to the vehicle controller
        self.main_timer = self.create_timer(timer_period_sec=0.1, callback=self.main_timer_callback)

        ### CAR1 VAR
        ###################################################################################################
        # define the variables that will store the data from the two message inputs
        self.current_goal_point_car1 = np.array([0.0, 0.0, 0.0])
        self.current_goal_heading_rad_car1 = 0.0
        self.closest_point_car1 = np.array([0.0, 0.0, 0.0])
        self.closest_heading_rad_car1 = 0.0
        self.speed_car1 = 0.0
        self.state_car1 = 0.0

        self.vehicle_point_car1 = np.array([0.0, 0.0, 0.0])
        self.vehicle_heading_rad_car1 = 0.0

        self.have_vehicle_pose_car1 = False
        self.have_goal_pose_car1 = False
        ###################################################################################################


        ### CAR2 VAR
        ###################################################################################################
        # define the variables that will store the data from the two message inputs
        self.current_goal_point_car2 = np.array([0.0, 0.0, 0.0])
        self.current_goal_heading_rad_car2 = 0.0
        self.closest_point_car2 = np.array([0.0, 0.0, 0.0])
        self.closest_heading_rad_car2 = 0.0
        self.speed_car2 = 0.0
        self.state_car2 = 0.0

        self.vehicle_point_car2 = np.array([0.0, 0.0, 0.0])
        self.vehicle_heading_rad_car2 = 0.0

        self.have_vehicle_pose_car2 = False
        self.have_goal_pose_car2 = False
        ###################################################################################################

        #self.error_file = open('error_file.txt','w')
        self.time_elapsed = 0
        self.pause = False

        ### END OF CONTRUCTOR
        ###################################################################################################

    def vehicle_pose_callback_car1(self, msg):
        self.have_vehicle_pose_car1 = True

        self.vehicle_point_car1[0] = msg.pose.position.x
        self.vehicle_point_car1[1] = msg.pose.position.y
        self.vehicle_point_car1[2] = 0.0
        self.vehicle_heading_rad_car1 = 2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
    def vehicle_pose_callback_car2(self, msg):
        self.have_vehicle_pose_car2 = True

        self.vehicle_point_car2[0] = msg.pose.position.x
        self.vehicle_point_car2[1] = msg.pose.position.y
        self.vehicle_point_car2[2] = 0.0
        self.vehicle_heading_rad_car2 = 2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
    
    def current_goal_pose_callback_car1(self, msg):
        self.have_goal_pose_car1 = True

        self.current_goal_point_car1[0] = msg.current_goal_pose.pose.position.x
        self.current_goal_point_car1[1] = msg.current_goal_pose.pose.position.y
        self.current_goal_point_car1[2] = 0.0
        self.current_goal_heading_rad_car1 = 2.0 * math.atan2(
            msg.current_goal_pose.pose.orientation.z, msg.current_goal_pose.pose.orientation.w
        )

        self.closest_point_car1[0] = msg.closest_pose.pose.position.x
        self.closest_point_car1[1] = msg.closest_pose.pose.position.y
        self.closest_point_car1[2] = 0.0
        self.closest_heading_rad_car1 = 2.0 * math.atan2(
            msg.closest_pose.pose.orientation.z, msg.closest_pose.pose.orientation.w
        )

        self.speed_car1 = msg.speed
        self.state_car1 = msg.state

        if self.speed_car1 > 2.5:
            self.speed_car1 = 2.5
        elif self.speed_car1 < -2.5:
            self.speed_car1 = -2.5
    def current_goal_pose_callback_car2(self, msg):
        self.have_goal_pose_car2 = True

        self.current_goal_point_car2[0] = msg.current_goal_pose.pose.position.x
        self.current_goal_point_car2[1] = msg.current_goal_pose.pose.position.y
        self.current_goal_point_car2[2] = 0.0
        self.current_goal_heading_rad_car2 = 2.0 * math.atan2(
            msg.current_goal_pose.pose.orientation.z, msg.current_goal_pose.pose.orientation.w
        )

        self.closest_point_car2[0] = msg.closest_pose.pose.position.x
        self.closest_point_car2[1] = msg.closest_pose.pose.position.y
        self.closest_point_car2[2] = 0.0
        self.closest_heading_rad_car2 = 2.0 * math.atan2(
            msg.closest_pose.pose.orientation.z, msg.closest_pose.pose.orientation.w
        )

        self.speed_car2 = msg.speed
        self.state_car2 = msg.state

        if self.speed_car2 > 2.5:
            self.speed_car2 = 2.5
        elif self.speed_car2 < -2.5:
            self.speed_car2 = -2.5

    def e_stop_callback(self, msg):
        if msg.data == 0:
            self.pause = True
        elif msg.data == 1:
            self.pause = False

    def main_timer_callback(self):

        if self.pause == True:
            # send out a zero velocity Ackermann message
            out_msg = AckermannDriveStamped()
            out_msg.drive.speed = 0.0
            out_msg.drive.steering_angle = 0.0

            self.publisher_car1.publish(out_msg)
            self.publisher_car2.publish(out_msg)
            return

        ## CAR1
        ##########################################################################################################
        # This will only publish after each subscription has occured at least once
        if self.have_goal_pose_car1 and self.have_vehicle_pose_car1:
            p1_ratio = 0.25

            error = math.sqrt(((self.closest_point_car1[0]-self.vehicle_point_car1[0])**2)+((self.closest_point_car1[1]-self.vehicle_point_car1[1])**2))
            angle_error = self.closest_heading_rad_car1 - self.vehicle_heading_rad_car1

            self.L_wheelbase_m = self.get_parameter("L_wheelbase_m").value

            out_msg = AckermannDriveStamped()
            out_msg.drive.speed = self.speed_car1
            #angle_rad = math.atan(self.L_wheelbase_m / radius_of_curvature)

            error_cross_track, error_heading_rad, line_pt = get_cross_track_and_heading_error(self.closest_point_car1,self.closest_heading_rad_car1,self.vehicle_point_car1,self.vehicle_heading_rad_car1)
            angle_rad = error_heading_rad + math.atan(error_cross_track*2.5/0.75)

            out_msg.drive.steering_angle = angle_rad

            #self.error_file.write(f'{self.time_elapsed},{angle_rad},{self.closest_heading_rad_car1},{self.vehicle_heading_rad_car1},{self.closest_point_car1[0]},{self.closest_point_car1[1]},{self.vehicle_point_car1[0]},{self.vehicle_point_car1[1]},{error_cross_track},{error_heading_rad}\n')
            self.time_elapsed += 0.1

            self.publisher_car1.publish(out_msg)
        ##########################################################################################################

        
        ## CAR2
        ##########################################################################################################
        # This will only publish after each subscription has occured at least once
        if self.have_goal_pose_car2 and self.have_vehicle_pose_car2:
            p1_ratio = 0.25

            error = math.sqrt(((self.closest_point_car2[0]-self.vehicle_point_car2[0])**2)+((self.closest_point_car2[1]-self.vehicle_point_car2[1])**2))
            angle_error = self.closest_heading_rad_car2 - self.vehicle_heading_rad_car2

            self.L_wheelbase_m = self.get_parameter("L_wheelbase_m").value

            out_msg = AckermannDriveStamped()
            out_msg.drive.speed = self.speed_car2
            #angle_rad = math.atan(self.L_wheelbase_m / radius_of_curvature)

            error_cross_track, error_heading_rad, line_pt = get_cross_track_and_heading_error(self.closest_point_car2,self.closest_heading_rad_car2,self.vehicle_point_car2,self.vehicle_heading_rad_car2)
            angle_rad = error_heading_rad + math.atan(error_cross_track*2.5/0.75)

            out_msg.drive.steering_angle = angle_rad

            #self.error_file.write(f'{self.time_elapsed},{angle_rad},{self.closest_heading_rad_car2},{self.vehicle_heading_rad_car2},{self.closest_point_car2[0]},{self.closest_point_car2[1]},{self.vehicle_point_car2[0]},{self.vehicle_point_car2[1]},{error_cross_track},{error_heading_rad}\n')
            self.time_elapsed += 0.1

            self.publisher_car2.publish(out_msg)
        ##########################################################################################################


def main(args=None):
    rclpy.init(args=args)

    vehicle_controller = VehicleController()

    rclpy.spin(vehicle_controller)
    
    vehicle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
