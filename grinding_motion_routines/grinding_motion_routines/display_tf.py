#!/usr/bin/env python3
# import rospy
import tf2_ros
import geometry_msgs.msg # TransformStamped, Transform
# numpy はこのファイルでは不要になりました
import rclpy
from rclpy.node import Node
# Pose, Quaternion, Vector3 は直接使われていません


class TFPublisher(Node):
    def __init__(self, parent_link="base_link", child_link="debug_tf_") -> None: # child_linkのデフォルト値を変更
        super().__init__("tf_publisher_node") # ノード名をより具体的に
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_stamped = geometry_msgs.msg.TransformStamped() # 変数名をtfから変更
        self.parent_link = parent_link
        self.child_link_prefix = child_link # child_link -> child_link_prefix

    def broadcast_tf_with_waypoints(
        self, waypoints: list # Type hint for waypoints
    ):
        """
        Broadcasts a TF for each waypoint in the list.
        Each waypoint is expected to be a list of 7 floats: [x,y,z,qx,qy,qz,qw].
        """
        if not waypoints:
            self.get_logger().warn("Waypoints list is empty. Nothing to broadcast.")
            return

        for index, pose_values in enumerate(waypoints):
            if not isinstance(pose_values, list) or len(pose_values) != 7:
                self.get_logger().error(
                    f"Waypoint at index {index} is not a list of 7 floats. Skipping."
                )
                continue
            
            # print(pose_values) # For debugging
            current_transform = geometry_msgs.msg.Transform()
            
            current_transform.translation.x = float(pose_values[0])
            current_transform.translation.y = float(pose_values[1])
            current_transform.translation.z = float(pose_values[2])
            current_transform.rotation.x = float(pose_values[3])
            current_transform.rotation.y = float(pose_values[4])
            current_transform.rotation.z = float(pose_values[5])
            current_transform.rotation.w = float(pose_values[6])

            # Construct the child frame ID using the prefix and index
            child_frame_id = self.child_link_prefix + str(index)
            
            self._broadcast_single_tf( # Renamed for clarity
                current_transform,
                self.parent_link,
                child_frame_id,
            )
        self.get_logger().info(f"Broadcasted TFs for {len(waypoints)} waypoints with prefix '{self.child_link_prefix}'.")

    def _broadcast_single_tf(self, transform_msg: geometry_msgs.msg.Transform, parent_link: str, child_link: str): # Renamed and type hints added
        self.tf_stamped.header.stamp = self.get_clock().now().to_msg()
        self.tf_stamped.header.frame_id = parent_link
        self.tf_stamped.child_frame_id = child_link
        self.tf_stamped.transform.translation = transform_msg.translation
        self.tf_stamped.transform.rotation = transform_msg.rotation
        self.broadcaster.sendTransform(self.tf_stamped)