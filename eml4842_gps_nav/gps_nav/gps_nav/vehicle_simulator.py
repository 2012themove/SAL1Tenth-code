import math
import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from ackermann_msgs.msg import AckermannDriveStamped

from gps_nav.uf_support.route_support import update_vehicle_pose

D2R = math.pi / 180.0
R2D = 1.0 / D2R

class VehicleSimulator(Node):
    def __init__(self):
        super().__init__("vehicle_simulator")

        ###car1 Specs
        ###################################################################################################
        self.declare_parameter("starting_position", [0.0, 0.0, 0.0])
        self.declare_parameter("starting_ang_deg", 190.0)
        self.declare_parameter("L_wheelbase_m", 0.33)

        self.subscription_twist = self.create_subscription(
            Twist, "vehicle_command_twist", self.vehicle_command_twist_callback, 1
        )

        self.subscription_angle = self.create_subscription(
            AckermannDriveStamped, "ackermann_cmd", self.vehicle_command_ackermann_callback, 1
        )

        self.publisher = self.create_publisher(PoseStamped, "/car1/pos", 10)

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        self.timer = self.create_timer(timer_period_sec=0.1, callback=self.timer_callback)

        param_value = self.get_parameter("starting_position").value

        self.old_position = np.array([0.0, 0.0, 0.0])
        self.old_position[0] = param_value[0]
        self.old_position[1] = param_value[1]
        self.old_position[2] = param_value[2]

        self.old_heading_rad = D2R * self.get_parameter("starting_ang_deg").value
        self.L_wheelbase_m = self.get_parameter("L_wheelbase_m").value

        self.rad_of_curvature = 99999.0
        self.speed = 0.0
        self.cnt = 0

        self.steering_angle_rad = 0.0
        self.throttle_effort = 0.0

        self.input_type = 0  # 0 is undefined; 1 is twist input ; 2 is effort (angle) input

        ###End of car1 stuff
        ###################################################################################################


        ###Car2 Specs
        ###################################################################################################
        self.declare_parameter("starting_position_car2", [0.0, 0.0, 0.0])
        self.declare_parameter("starting_ang_deg_car2", 190.0)

        self.subscription_twist_car2 = self.create_subscription(
            Twist, "vehicle_command_twist_car2", self.vehicle_command_twist_callback_car2, 1
        )

        self.subscription_angle_car2 = self.create_subscription(
            AckermannDriveStamped, "car2/ackermann_cmd", self.vehicle_command_ackermann_callback_car2, 1
        )

        self.publisher_car2 = self.create_publisher(PoseStamped, "/car2/pos", 10)

        param_value_car2 = self.get_parameter("starting_position_car2").value

        self.old_position_car2 = np.array([0.0, 0.0, 0.0])
        self.old_position_car2[0] = param_value_car2[0]
        self.old_position_car2[1] = param_value_car2[1]
        self.old_position_car2[2] = param_value_car2[2]

        self.old_heading_rad_car2 = D2R * self.get_parameter("starting_ang_deg_car2").value

        self.rad_of_curvature_car2 = 99999.0
        self.speed_car2 = 0.0
        self.cnt_car2 = 0

        self.steering_angle_rad_car2 = 0.0
        self.throttle_effort_car2 = 0.0

        self.input_type_car2 = 0
        ###################################################################################################
        #End of constructor
        

    def vehicle_command_twist_callback(self, msg):

        self.input_type = 1

        if math.isclose(msg.angular.z, 0.0, abs_tol=0.0001) or math.isclose(msg.linear.x, 0.0, abs_tol=0.001):
            self.rad_of_curvature = 999999.9
        else:
            self.rad_of_curvature = msg.linear.x / msg.angular.z

        self.speed = msg.linear.x
        self.cnt += 1

    def vehicle_command_twist_callback_car2(self, msg):

        self.input_type_car2 = 1

        if math.isclose(msg.angular.z, 0.0, abs_tol=0.0001) or math.isclose(msg.linear.x, 0.0, abs_tol=0.001):
            self.rad_of_curvature_car2 = 999999.9
        else:
            self.rad_of_curvature_car2 = msg.linear.x / msg.angular.z

        self.speed_car2 = msg.linear.x
        self.cnt_car2 += 1


    def vehicle_command_ackermann_callback(self, msg):

        self.input_type = 2

        self.steering_angle_rad = msg.drive.steering_angle
        self.throttle_effort = (msg.drive.speed/6.35) *100
        self.cnt += 1

    def vehicle_command_ackermann_callback_car2(self, msg):

        self.input_type_car2 = 2

        self.steering_angle_rad_car2 = msg.drive.steering_angle
        self.throttle_effort_car2 = (msg.drive.speed/6.35) *100
        self.cnt_car2 += 1


    def timer_callback(self):
        #CAR1 SPEC
        # PN - Is this here because the parameter might not be updated by the time a control is received?
        # CC - At the start, the initial 'old_heading_rad' must be  set to some value
        if self.cnt < 1:
            self.old_heading_rad = D2R * self.get_parameter("starting_ang_deg").value

        if self.input_type == 2:
            # get radius of curvature and speed from VehCmd message info
            if np.isclose(self.steering_angle_rad, 0.0, atol=1.0e-5):
                self.rad_of_curvature = 1.0e+5
            else:
                self.rad_of_curvature = self.L_wheelbase_m / math.tan(self.steering_angle_rad)
            self.speed = self.throttle_effort * 4.0 / 100.0  # max speed is 4 m/sec

        new_position, new_heading_rad = update_vehicle_pose(
            self.old_position, self.old_heading_rad, self.rad_of_curvature, self.speed / 10.0
        )

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "vehicle_viz"
        msg.pose.position.x = new_position[0]
        msg.pose.position.y = new_position[1]
        msg.pose.position.z = new_position[2]
        msg.pose.orientation.w = math.cos(new_heading_rad / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(new_heading_rad / 2.0)

        self.publisher.publish(msg)

        # send out the tf2 for the vehicle pose
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = "vehicle_viz"
        t.child_frame_id = "car1"

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        # Send the transformation
        self.br.sendTransform(t)

        self.old_position = new_position
        self.old_heading_rad = new_heading_rad

        ##END OF CAR1
        ##############################################################################################
        ##CAR2 SPEC
        if self.cnt_car2 < 1:
            self.old_heading_rad_car2 = D2R * self.get_parameter("starting_ang_deg_car2").value

        if self.input_type_car2 == 2:
            # get radius of curvature and speed from VehCmd message info
            if np.isclose(self.steering_angle_rad_car2, 0.0, atol=1.0e-5):
                self.rad_of_curvature_car2 = 1.0e+5
            else:
                self.rad_of_curvature_car2 = self.L_wheelbase_m / math.tan(self.steering_angle_rad_car2)
            self.speed_car2 = self.throttle_effort_car2 * 4.0 / 100.0  # max speed is 4 m/sec

        new_position, new_heading_rad = update_vehicle_pose(
            self.old_position_car2, self.old_heading_rad_car2, self.rad_of_curvature_car2, self.speed_car2 / 10.0
        )

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "vehicle_viz"
        msg.pose.position.x = new_position[0]
        msg.pose.position.y = new_position[1]
        msg.pose.position.z = new_position[2]
        msg.pose.orientation.w = math.cos(new_heading_rad / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(new_heading_rad / 2.0)

        self.publisher_car2.publish(msg)

        # send out the tf2 for the vehicle pose
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = "vehicle_viz"
        t.child_frame_id = "car2"

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        # Send the transformation
        self.br.sendTransform(t)

        self.old_position_car2 = new_position
        self.old_heading_rad_car2 = new_heading_rad


def main(args=None):
    rclpy.init(args=args)

    vehicle_simulator = VehicleSimulator()

    rclpy.spin(vehicle_simulator)

    vehicle_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
