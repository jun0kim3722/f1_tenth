#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import os
import time

def read_waypoints_from_file(filename):
    """
    Reads waypoints from a specified file and returns a list of tuples (x, y, yaw).
    """
    waypoints = []
    with open(filename, 'r') as file:
        for line in file:
            # Split the line and convert values to float
            x, y, yaw = map(float, line.strip().strip('[]').split(','))
            waypoints.append((x, y, yaw))
    return waypoints

def file_was_modified(filename, last_modified_time):
    """
    Check if the file was modified since the last known modification time.
    """
    return os.path.getmtime(filename) > last_modified_time

def publish_waypoints(filename):
    """
    Publishes waypoints as a LINE_STRIP marker for visualization in RViz,
    updates waypoints if the file changes.
    """
    rospy.init_node('waypoint_publisher', anonymous=True)
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    last_modified_time = 0  # Initialize last modified time to zero

    while not rospy.is_shutdown():
        # Check if file was modified since last read
        if file_was_modified(filename, last_modified_time):
            waypoints = read_waypoints_from_file(filename)
            last_modified_time = os.path.getmtime(filename)

            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0  # Identity quaternion
            marker.scale.x = 0.05  # Line width
            marker.color.a = 1.0  # Alpha (opacity)
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0  # Green
            marker.color.b = 0.0  # Blue

            # Populate the points of the LINE_STRIP
            marker.points = []
            for x, y, yaw in waypoints:
                p = Point()
                p.x = x
                p.y = y
                marker.points.append(p)

        pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    filename = '/home/nvidia/rallycar_ws/src/purdue_power_pistons_lab6/waypoints/waypoints.txt'
    publish_waypoints(filename)

