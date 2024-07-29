#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
import math
import numpy as np
from visualization_msgs.msg import Marker
import pdb

class MapFollower:
    def __init__(self):
        rospy.init_node('map_follower')
        self.waypoints = None
        self.current_pose = None
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.path_subscriber = rospy.Subscriber('/desired_path', Path, self.load_waypoints)
        self.steer_pub = rospy.Publisher('/steering_cmd', Float32, queue_size=1)
        self.throttle_pub = rospy.Publisher('/accelerator_cmd', Float32, queue_size=1)
        self.marker_pub = rospy.Publisher('waypoint_marker', Marker, queue_size=10)
        self.current_waypoint_index = 0
        self.Kp_orientation = 1.2
        self.Ki_orientation = 0.01
        self.Kd_orientation = 0.9
        self.previous_error_orientation = 0
        self.integral_error_orientation = 0
        self.rate = rospy.Rate(100)

    def load_waypoints(self, data):
        if self.waypoints is not None:
                return
        way_point_list = data.poses
        waypoints = []
        for pos in way_point_list:
            waypoints.append([pos.pose.position.x, pos.pose.position.y, 0])

        self.waypoints = np.array(waypoints)
        rospy.loginfo("Waypoints loaded")
        '''
        try:
            waypoints = np.loadtxt('/home/nvidia/rallycar_ws/src/purdue_power_pistons_lab6/waypoints/waypoints.txt', delimiter=',')
            self.waypoints = waypoints
            rospy.loginfo("Waypoints successfully loaded.")
        except Exception as e:
            rospy.logerr(f"Failed to load waypoints: {e}")
            rospy.signal_shutdown("Failed to load waypoints")
            return
        '''
    def pose_callback(self, data):
        print("pos print!!!!!!!!!!!!!!\n", data)
        self.current_pose = data

    def scan_callback(self, data):
        if self.current_pose is None or self.waypoints is None:
            rospy.loginfo("Waiting for pose and waypoints to be available.")
            print("pos:", self.current_pose)
            return

        distance_to_waypoint, orientation_error = self.calculate_errors()
        steer_command = self.calculate_steer_command(orientation_error)
        throttle_command = self.calculate_throttle_command(distance_to_waypoint, orientation_error)

        rospy.logdebug(f"Steer Command: {steer_command}, Throttle Command: {throttle_command}")

        self.publish_waypoint_marker()
        self.steer_pub.publish(Float32(steer_command))
        self.throttle_pub.publish(Float32(throttle_command))

    def calculate_errors(self):
        current_x, current_y, current_theta = self.extract_pose(self.current_pose)
        waypoint_x, waypoint_y = self.waypoints[self.current_waypoint_index][:2]
        dx = waypoint_x - current_x
        dy = waypoint_y - current_y
        distance_to_waypoint = math.sqrt(dx**2 + dy**2)
        angle_to_waypoint = math.atan2(dy, dx)
        orientation_error = self.wrap_to_pi(angle_to_waypoint - current_theta)

        return distance_to_waypoint, orientation_error

    def calculate_steer_command(self, orientation_error):
        self.integral_error_orientation += orientation_error
        delta_error_orientation = orientation_error - self.previous_error_orientation
        angular_velocity = (self.Kp_orientation * orientation_error +
                             self.Ki_orientation * self.integral_error_orientation +
                             self.Kd_orientation * delta_error_orientation)
        steering_command = angular_velocity * 2048 * 2 / math.pi
        self.previous_error_orientation = orientation_error

        steering_command = max(min(steering_command, 2048), -2048)
        if abs(steering_command) == 2048:
            rospy.loginfo("Steering command saturated. Preventing integral wind-up.")
            if abs(orientation_error) < 0.1:
                self.integral_error_orientation = 0

        return steering_command

    def calculate_throttle_command(self, distance_to_waypoint, orientation_error):
        if distance_to_waypoint < 0.6:
            rospy.loginfo(f"Reached Waypoint {self.current_waypoint_index}")
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                rospy.loginfo("All waypoints reached. Stopping.")
                rospy.signal_shutdown("Finished all waypoints")
            return 0
        
        # Check for sharp turns and adjust speed
        if abs(orientation_error) > 0.5:  # Adjust threshold based on required turn sensitivity
            return 350  # Reduce speed for sharp turns
        elif distance_to_waypoint > 1.7:
            return 550  # Increase speed if far from waypoint
        else:
            return 450  # Intermediate speed

    def extract_pose(self, pose):
        x = pose.pose.pose.position.x
        y = pose.pose.pose.position.y
        orientation = pose.pose.pose.orientation
        _, _, theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return x, y, theta

    def wrap_to_pi(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def publish_waypoint_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.position.x = self.waypoints[self.current_waypoint_index, 0]
        marker.pose.position.y = self.waypoints[self.current_waypoint_index, 1]
        marker.pose.position.z = 0
        self.marker_pub.publish(marker)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        follower = MapFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass

