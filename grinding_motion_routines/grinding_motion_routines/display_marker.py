#!/usr/bin/env python3
from rclpy.node import Node
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


from tf_transformations import quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import Quaternion, Pose
from builtin_interfaces.msg import Duration
import numpy as np
import time

class DisplayMarker(Node):
    def __init__(self, marker_publisher_name="debug_marker"):
        super().__init__("marker_display")
        self.publisher = self.create_publisher(MarkerArray, marker_publisher_name, 1)
        self.index = 0
        
    def wait_for_connection(self):
        self.get_logger().info(f'Waiting for a subscriber to connect to the marker publisher on topic "{self.publisher.topic_name}"')
        while(self.publisher.get_subscription_count() == 0):
          time.sleep(1)
          counter=self.publisher.get_subscription_count()
        self.get_logger().info(f'Subscriber connected to the marker publisher on topic "{self.publisher.topic_name}"!')


        
    def display_waypoints(self, waypoints, scale=0.002, type=None):
        marker_array = MarkerArray()

        if type is None:
            type = Marker.SPHERE
        for index, points in enumerate(waypoints):
            # pointsの型がPoseでなければ変換
            if not isinstance(points, Pose):
                point_pose = Pose()
                point_pose.position.x = points[0]
                point_pose.position.y = points[1]
                point_pose.position.z = points[2]
                point_pose.orientation.x = points[3]
                point_pose.orientation.y = points[4]
                point_pose.orientation.z = points[5]
                point_pose.orientation.w = points[6]
                points = point_pose
                
            marker = Marker()

            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            self.index += 1
            marker.id = self.index
            marker.action = Marker.ADD

            marker.pose.position = points.position
            marker.pose.orientation = points.orientation
            red_strength= 1.0*index/(len(waypoints)-1)
            blue_strength= 1.0*(len(waypoints)-index-1)/(len(waypoints)-1)
            marker.color.r = red_strength
            marker.color.g = 0.0
            marker.color.b = blue_strength
            marker.color.a = 1.0
            if type == marker.ARROW:
                marker.scale.x = scale
                marker.scale.y = scale * 0.1
                marker.scale.z = scale * 0.1

                quat = [
                    marker.pose.orientation.x,
                    marker.pose.orientation.y,
                    marker.pose.orientation.z,
                    marker.pose.orientation.w,
                ]
                q_orig = np.array(quat)
                q_rot = quaternion_from_euler(0, np.pi / 2, 0)
                q_new = quaternion_multiply(q_rot, q_orig)
                marker.pose.orientation = Quaternion(
                    x=q_new[0], y=q_new[1], z=q_new[2], w=q_new[3]
                )
            else:
                marker.scale.x = scale
                marker.scale.y = scale
                marker.scale.z = scale

            marker.type = type
            marker.lifetime = Duration(sec=60)  # existing marker for ever
            marker_array.markers.append(marker)
        self.wait_for_connection()
        self.publisher.publish(marker_array)
        self.get_logger().info("Published!")
