import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from gps_nav_interfaces.srv import GetRoutePoses
from gps_nav_interfaces.msg import CurrentGoalPose, LookAheadSpecs

import gps_nav.uf_support.route_support as uf_nav

route_poses = []

class GoalPoseCreator(Node):

    def __init__(self):
        super().__init__('goal_pose_creator')
        self.cli1 = self.create_client(GetRoutePoses, 'get_route_poses_car1')
        while not self.cli1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service 1 not available, waiting again...')
        self.req1 = GetRoutePoses.Request()

        self.cli2 = self.create_client(GetRoutePoses, 'get_route_poses_car2')
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service 2 not available, waiting again...')
        self.req2 = GetRoutePoses.Request()

        self.declare_parameter("distBetweenPoints", 0.05)
        self.dist_between_pts = self.get_parameter("distBetweenPoints").value


        # subscribe to 'vehicle_pose' topic
        self.subscription_vehicle1_pose = self.create_subscription(
            PoseStamped, '/car1/pos', self.vehicle1_pose_callback, 1)
        self.subscription_vehicle2_pose = self.create_subscription(
            PoseStamped, '/car2/pos', self.vehicle2_pose_callback, 1)

        # subscribe to 'look_ahead_specs' topic
        self.subscription_look_ahead_specs = self.create_subscription(
            LookAheadSpecs, 'look_ahead_specs', self.look_ahead_specs_callback, 10)

        # prepare to publish 'current_goal_pose' topic
        self.publisher_current_goal_pose_car1 = self.create_publisher(
            CurrentGoalPose, 'car1/current_goal_pose', 10)
        self.publisher_current_goal_pose_car2 = self.create_publisher(
            CurrentGoalPose, 'car2/current_goal_pose', 10)

        # define variables
        self.speed = 0.0            # meters/second
        self.look_ahead_dist = 8.0  # meters
        self.have_look_ahead_specs = False

        ##CAR1
        #################################################################################################
        self.g_xUTM_car1 = 0.0           # meters     
        self.g_yUTM_car1 = 0.0           # meters
        self.num_route_segments_car1 = 0
        self.route_segments_car1 = []
        self.look_ahead_pose_car1 = uf_nav.route_pose_class()
        self.closest_pose_car1 = uf_nav.route_pose_class()
        self.current_seg_num_car1 = 0
        self.look_ahead_seg_num_car1 = 0
        self.stop_flag_car1 = False
        self.ready_to_process_car1 = False
        self.have_vehicle_pose_car1 = False
        self.want_loop_car1 = False
        #################################################################################################

        ##CAR2
        #################################################################################################
        self.g_xUTM_car2 = 0.0           # meters     
        self.g_yUTM_car2 = 0.0           # meters
        self.num_route_segments_car2 = 0
        self.route_segments_car2 = []
        self.look_ahead_pose_car2 = uf_nav.route_pose_class()
        self.closest_pose_car2 = uf_nav.route_pose_class()
        self.current_seg_num_car2 = 0
        self.look_ahead_seg_num_car2 = 0
        self.stop_flag_car2 = False
        self.ready_to_process_car2 = False
        self.have_vehicle_pose_car2 = False
        self.want_loop_car2 = False
        #################################################################################################

    def send_request_car1(self):  # no data is sent in the request
        self.future1 = self.cli1.call_async(self.req1)
    def send_request_car2(self):  # no data is sent in the request
        self.future2 = self.cli2.call_async(self.req2)

    def look_ahead_specs_callback(self, msg):
        self.have_look_ahead_specs = True
        self.speed = msg.speed
        self.look_ahead_dist = msg.look_ahead_dist

    def vehicle1_pose_callback(self, msg):
        if not self.ready_to_process_car1:
            return
        
        self.have_vehicle_pose_car1 = True
        self.g_xUTM_car1 = msg.pose.position.x
        self.g_yUTM_car1 = msg.pose.position.y

        vehicle_pt = np.array([self.g_xUTM_car1, self.g_yUTM_car1, 0.0])

        ans = uf_nav.get_look_ahead_point_v2(
            self.look_ahead_dist, vehicle_pt, self.route_segments_car1, self.current_seg_num_car1)

        self.look_ahead_pose_car1 = ans[0]
        self.closest_pose_car1 = ans[1]
        self.current_seg_num_car1 = ans[2]
        self.look_ahead_seg_num_car1 = ans[3]
        self.stop_flag_car1 = ans[4]

        # publish the current_goal_pose topic message
        out_msg = CurrentGoalPose()
        out_msg.current_goal_pose.header.frame_id = 'utm'
        out_msg.current_goal_pose.header.stamp = self.get_clock().now().to_msg()
        out_msg.current_goal_pose.pose.position.x = self.look_ahead_pose_car1.pt[0]
        out_msg.current_goal_pose.pose.position.y = self.look_ahead_pose_car1.pt[1]
        out_msg.current_goal_pose.pose.position.z = self.look_ahead_pose_car1.pt[2]
        out_msg.current_goal_pose.pose.orientation.w = math.cos(
            self.look_ahead_pose_car1.heading_rad/2.0)
        out_msg.current_goal_pose.pose.orientation.x = 0.0
        out_msg.current_goal_pose.pose.orientation.y = 0.0
        out_msg.current_goal_pose.pose.orientation.z = math.sin(
            self.look_ahead_pose_car1.heading_rad/2.0)

        out_msg.closest_pose.header.frame_id = 'utm'
        out_msg.closest_pose.header.stamp = out_msg.current_goal_pose.header.stamp
        out_msg.closest_pose.pose.position.x = self.closest_pose_car1.pt[0]
        out_msg.closest_pose.pose.position.y = self.closest_pose_car1.pt[1]
        out_msg.closest_pose.pose.position.z = self.closest_pose_car1.pt[2]

        heading_at_closest_rad = uf_nav.get_heading_rad_at_u(self.route_segments_car1[self.current_seg_num_car1], 0.0)
        out_msg.closest_pose.pose.orientation.w = math.cos(heading_at_closest_rad/2.0)
        out_msg.closest_pose.pose.orientation.x = 0.0
        out_msg.closest_pose.pose.orientation.y = 0.0
        out_msg.closest_pose.pose.orientation.z = math.sin(heading_at_closest_rad/2.0)

        out_msg.closest_pose.pose.orientation.w = math.cos(
            self.closest_pose_car1.heading_rad/2.0)
        out_msg.closest_pose.pose.orientation.x = 0.0
        out_msg.closest_pose.pose.orientation.y = 0.0
        out_msg.closest_pose.pose.orientation.z = math.sin(
            self.closest_pose_car1.heading_rad/2.0)

        if(self.want_loop_car1 == False and self.current_seg_num_car1 == self.num_route_segments_car1-1):
            out_msg.speed = 0.0
        else:
            out_msg.speed = self.speed

        self.publisher_current_goal_pose_car1.publish(out_msg)

    def vehicle2_pose_callback(self, msg):
        if not self.ready_to_process_car2:
            return
        
        self.have_vehicle_pose_car2 = True
        self.g_xUTM_car2 = msg.pose.position.x
        self.g_yUTM_car2 = msg.pose.position.y

        vehicle_pt = np.array([self.g_xUTM_car2, self.g_yUTM_car2, 0.0])
        
        print(self.route_segments_car1)
        print(self.route_segments_car2)
        
        ans = uf_nav.get_look_ahead_point_v2(
            self.look_ahead_dist, vehicle_pt, self.route_segments_car2, self.current_seg_num_car2)

        self.look_ahead_pose_car2 = ans[0]
        self.closest_pose_car2 = ans[1]
        self.current_seg_num_car2 = ans[2]
        self.look_ahead_seg_num_car2 = ans[3]
        self.stop_flag_car2 = ans[4]

        # publish the current_goal_pose topic message
        out_msg = CurrentGoalPose()
        out_msg.current_goal_pose.header.frame_id = 'utm'
        out_msg.current_goal_pose.header.stamp = self.get_clock().now().to_msg()
        out_msg.current_goal_pose.pose.position.x = self.look_ahead_pose_car2.pt[0]
        out_msg.current_goal_pose.pose.position.y = self.look_ahead_pose_car2.pt[1]
        out_msg.current_goal_pose.pose.position.z = self.look_ahead_pose_car2.pt[2]
        out_msg.current_goal_pose.pose.orientation.w = math.cos(
            self.look_ahead_pose_car2.heading_rad/2.0)
        out_msg.current_goal_pose.pose.orientation.x = 0.0
        out_msg.current_goal_pose.pose.orientation.y = 0.0
        out_msg.current_goal_pose.pose.orientation.z = math.sin(
            self.look_ahead_pose_car2.heading_rad/2.0)

        out_msg.closest_pose.header.frame_id = 'utm'
        out_msg.closest_pose.header.stamp = out_msg.current_goal_pose.header.stamp
        out_msg.closest_pose.pose.position.x = self.closest_pose_car2.pt[0]
        out_msg.closest_pose.pose.position.y = self.closest_pose_car2.pt[1]
        out_msg.closest_pose.pose.position.z = self.closest_pose_car2.pt[2]

        heading_at_closest_rad = uf_nav.get_heading_rad_at_u(self.route_segments_car2[self.current_seg_num_car2], 0.0)
        out_msg.closest_pose.pose.orientation.w = math.cos(heading_at_closest_rad/2.0)
        out_msg.closest_pose.pose.orientation.x = 0.0
        out_msg.closest_pose.pose.orientation.y = 0.0
        out_msg.closest_pose.pose.orientation.z = math.sin(heading_at_closest_rad/2.0)

        out_msg.closest_pose.pose.orientation.w = math.cos(
            self.closest_pose_car2.heading_rad/2.0)
        out_msg.closest_pose.pose.orientation.x = 0.0
        out_msg.closest_pose.pose.orientation.y = 0.0
        out_msg.closest_pose.pose.orientation.z = math.sin(
            self.closest_pose_car2.heading_rad/2.0)

        if(self.want_loop_car2 == False and self.current_seg_num_car2 == self.num_route_segments_car2-1):
            out_msg.speed = 0.0
        else:
            out_msg.speed = self.speed

        self.publisher_current_goal_pose_car2.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)

    goal_pose_creator = GoalPoseCreator()
    goal_pose_creator.send_request_car1()
    goal_pose_creator.send_request_car2()

    route_publish_one = False
    route_publish_two = False

    while rclpy.ok():
        rclpy.spin_once(goal_pose_creator)
        ## CAR2
        ################################################################################################        
        if goal_pose_creator.future2.done() and (not route_publish_two):
            try:
                response = goal_pose_creator.future2.result()
            except Exception as e:
                goal_pose_creator.get_logger().warn(
                    'Service call car2 failed %r' % (e,))
            else:
                num_poses = response.num_route_poses
                last_pose = response.mypose[num_poses-1]
                goal_pose_creator.get_logger().info(
                    f"Goal Pose Creator car2 received {num_poses} poses. ")

                # create the route_poses array
                for i in range(num_poses):
                    ptx = response.mypose[i].position.x
                    pty = response.mypose[i].position.y
                    ptz = response.mypose[i].position.z
                    qw = response.mypose[i].orientation.w
                    qx = response.mypose[i].orientation.x
                    qy = response.mypose[i].orientation.y
                    qz = response.mypose[i].orientation.z
                    mystate = response.state[i]
                    myheadingrad = 2.0*math.atan2(qz, qw)
                    myw1 = 1.0
                    myw2 = 1.0
                    route_poses.append(uf_nav.route_pose_class(
                        np.array([ptx, pty, ptz]), myheadingrad, mystate, myw1, myw2))
                    
                # create the route_segments array
                goal_pose_creator.want_loop_car2 = response.want_loop
                goal_pose_creator.route_segments_car2 = uf_nav.create_route_segments(route_poses, goal_pose_creator.want_loop_car2, goal_pose_creator.dist_between_pts)

                goal_pose_creator.get_logger().info('Goal Pose creator car2 made %d route segments.' %
                                                 len(goal_pose_creator.route_segments_car2))

                goal_pose_creator.num_route_segments_car2 = num_poses

                # now add the heading values at each u value for each segment
                for seg in goal_pose_creator.route_segments_car2:
                    for i in np.arange(len(seg.pt_info)):
                        seg.pt_info[i,3] = seg.get_heading_rad(seg.pt_info[i,0])
                    
                goal_pose_creator.ready_to_process_car2 = True
                goal_pose_creator.get_logger().info('Ready to proceed with car2.')

                #outfile = open('equally_spaced.csv', 'w')
                #for seg in goal_pose_creator.route_segments:
                #    for j in np.arange(len(seg.pt_info)):
                #        print(f'{seg.pt_info[j,0]}, {seg.pt_info[j,1]}, {seg.pt_info[j,2]}, {seg.pt_info[j,3]}, {seg.pt_info[j,4]}', file = outfile)
                #outfile.close()

            route_publish_two = True
        ################################################################################################

        ## CAR1
        ################################################################################################
        if goal_pose_creator.future1.done() and (not route_publish_one):
            try:
                response = goal_pose_creator.future1.result()
            except Exception as e:
                goal_pose_creator.get_logger().warn(
                    'Service call car1 failed %r' % (e,))
            else:
                num_poses = response.num_route_poses
                last_pose = response.mypose[num_poses-1]
                goal_pose_creator.get_logger().info(
                    f"Goal Pose Creator car1 received {num_poses} poses. ")

                # create the route_poses array
                for i in range(num_poses):
                    ptx = response.mypose[i].position.x
                    pty = response.mypose[i].position.y
                    ptz = response.mypose[i].position.z
                    qw = response.mypose[i].orientation.w
                    qx = response.mypose[i].orientation.x
                    qy = response.mypose[i].orientation.y
                    qz = response.mypose[i].orientation.z
                    mystate = response.state[i]
                    myheadingrad = 2.0*math.atan2(qz, qw)
                    myw1 = 1.0
                    myw2 = 1.0
                    route_poses.append(uf_nav.route_pose_class(
                        np.array([ptx, pty, ptz]), myheadingrad, mystate, myw1, myw2))
                    
                # create the route_segments array
                goal_pose_creator.want_loop_car1 = response.want_loop
                goal_pose_creator.route_segments_car1 = uf_nav.create_route_segments(route_poses, goal_pose_creator.want_loop_car1, goal_pose_creator.dist_between_pts)

                goal_pose_creator.get_logger().info('Goal Pose creator car1 made %d route segments.' %
                                                 len(goal_pose_creator.route_segments_car1))

                goal_pose_creator.num_route_segments_car1 = num_poses

                # now add the heading values at each u value for each segment
                for seg in goal_pose_creator.route_segments_car1:
                    for i in np.arange(len(seg.pt_info)):
                        seg.pt_info[i,3] = seg.get_heading_rad(seg.pt_info[i,0])
                    
                goal_pose_creator.ready_to_process_car1 = True
                goal_pose_creator.get_logger().info('Ready to proceed with car1.')

                #outfile = open('equally_spaced.csv', 'w')
                #for seg in goal_pose_creator.route_segments:
                #    for j in np.arange(len(seg.pt_info)):
                #        print(f'{seg.pt_info[j,0]}, {seg.pt_info[j,1]}, {seg.pt_info[j,2]}, {seg.pt_info[j,3]}, {seg.pt_info[j,4]}', file = outfile)
                #outfile.close()

            route_publish_one = True
        ################################################################################################

        
        if route_publish_one and route_publish_two:
            break

    rclpy.spin(goal_pose_creator)

    goal_pose_creator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()