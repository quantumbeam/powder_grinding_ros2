#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose


class PosePublisher(Node):
    def __init__(self, node_name="pose_publisher_node", topic_name="debug_poses", frame_id="base_link") -> None:
        super().__init__(node_name)
        self.publisher = self.create_publisher(PoseArray, topic_name, 10)
        self.frame_id = frame_id

    def publish_poses_from_waypoints(
        self, waypoints: list
    ):
        """
        Publishes a PoseArray message containing all waypoints.
        Each waypoint is expected to be a list of 7 floats: [x,y,z,qx,qy,qz,qw].
        """
        # waypointsがnumpy arrayの場合はlistに変換
        import numpy as np
        if isinstance(waypoints, np.ndarray):
            waypoints = waypoints.tolist()
            
        if len(waypoints) == 0:
            self.get_logger().warn("Waypoints list is empty. Nothing to publish.")
            return

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self.frame_id
        
        for index, pose_values in enumerate(waypoints):
            # pose_valuesがnumpy arrayの場合はlistに変換
            if isinstance(pose_values, np.ndarray):
                pose_values = pose_values.tolist()
                
            if not isinstance(pose_values, list) or len(pose_values) != 7:
                self.get_logger().error(
                    f"Waypoint at index {index} is not a list of 7 floats. Skipping."
                )
                continue
            
            pose = Pose()
            pose.position.x = float(pose_values[0])
            pose.position.y = float(pose_values[1])
            pose.position.z = float(pose_values[2])
            pose.orientation.x = float(pose_values[3])
            pose.orientation.y = float(pose_values[4])
            pose.orientation.z = float(pose_values[5])
            pose.orientation.w = float(pose_values[6])
            
            pose_array.poses.append(pose)
            
        self.publisher.publish(pose_array)
        self.get_logger().info(f"Published PoseArray with {len(pose_array.poses)} poses to topic '{self.publisher.topic_name}'.")