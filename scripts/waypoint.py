#!/usr/bin/env python3

import rospy
import os
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path

class WaypointSaver(object):
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('waypoint_saver', anonymous=True)

        # Initialize waypoints list
        self.waypoints = []

        # Initialize ROS publishers
        self.marker_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.path_publisher = rospy.Publisher('visualization_path', Path, queue_size=10)
        
        # Subscribe to move_base_simple/goal topic
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.move_base_goal_callback)
         
        # Get file path for saving waypoints
        package_path = rospy.get_param('~package_path', os.path.join(os.path.expanduser('~'), 'rallycar_ws', 'src', 'rally_car'))
        scripts_path = os.path.join(package_path, 'scripts', 'waypoints.txt')
        self.file_path = rospy.get_param('~waypoints_file', scripts_path)

        # Create directory if it doesn't exist
        if not os.path.exists(os.path.dirname(self.file_path)):
            os.makedirs(os.path.dirname(self.file_path))
            rospy.loginfo("Directory for saving waypoints created at: {}".format(os.path.dirname(self.file_path)))

    def move_base_goal_callback(self, msg):
        # Extract waypoint from MoveBaseGoal message and store it
        x, y = msg.pose.position.x, msg.pose.position.y
        theta = 0  # Default value if theta is not provided in the message
        self.waypoints.append((x, y, theta))
        rospy.loginfo(f"Waypoint added: x={x}, y={y}, theta={theta}")
        self.save_waypoints()  # Save waypoints immediately after adding
        self.update_markers()
        self.publish_path()

    def update_markers(self):
        # Update visualization markers for waypoints
        marker_array = MarkerArray()
        for idx, (x, y, _) in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.id = idx
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)

    def publish_path(self):
        # Publish visualization path for waypoints
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for x, y, _ in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0  # no rotation, as theta is not visualized here
            path.poses.append(pose)
        self.path_publisher.publish(path)

    def save_waypoints(self):
        # Save waypoints to file
        try:
            with open(self.file_path, 'w') as file:
                for x, y, theta in self.waypoints:
                    file.write(f"{x}, {y}, {theta}\n")
            rospy.loginfo(f"Waypoints saved to {self.file_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save waypoints: {e}")

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    # Initialize WaypointSaver instance
    ws = WaypointSaver()
    
    # Save waypoints on node shutdown
    rospy.on_shutdown(ws.save_waypoints)
    
    # Start the node
    ws.run()

