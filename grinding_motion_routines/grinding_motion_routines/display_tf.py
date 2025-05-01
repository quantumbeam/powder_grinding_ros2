#!/usr/bin/env python3
# import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Quaternion,Vector3
import numpy as np
import rclpy
from rclpy.node import Node


class TFPublisher(Node):
    def __init__(self,waypoints,parent_link="base_link", child_link="debug_") -> None:
        super().__init__("tf_display")
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf = geometry_msgs.msg.TransformStamped()
        self.waypoints = waypoints
        self.parent_link = parent_link
        self.child_link = child_link
        self.create_timer(0.1, self.broadcast_tf_with_waypoints)


    def broadcast_tf_with_waypoints(
        self
    ):
        # rate = rospy.Rate(10)
        for index, pose in enumerate(self.waypoints):
            print(pose)
            pub_trans = Vector3()
            pub_trans.x = pose[0]
            pub_trans.y = pose[1]
            pub_trans.z = pose[2]
            pub_rot = Quaternion()
            pub_rot.x = pose[3]
            pub_rot.y = pose[4]
            pub_rot.z = pose[5]
            pub_rot.w = pose[6]

            self.breadcast_tf(
                pub_trans,
                pub_rot,
                self.parent_link,
                self.child_link + str(index),
            )
        # rate.sleep()
    
            # rate.sleep()

    # def broadcast_tf_with_pose(self, pose, parent_link="base_link", child_link="debug"):
    #     pub_pose = Pose()
    #     pub_pose.position.x = pose[0]
    #     pub_pose.position.y = pose[1]
    #     pub_pose.position.z = pose[2]
    #     pub_pose.orientation.x = pose[3]
    #     pub_pose.orientation.y = pose[4]
    #     pub_pose.orientation.z = pose[5]
    #     pub_pose.orientation.w = pose[6]
    #     self.breadcast_tf(
    #         pub_pose.position, pub_pose.orientation, parent_link, child_link
    #     )

    def breadcast_tf(self, tf_pos, tf_rot, parent_link, child_link):
        self.tf.header.stamp = self.get_clock().now().to_msg()
        self.tf.header.frame_id = parent_link
        self.tf.child_frame_id = child_link
        self.tf.transform.translation = tf_pos
        self.tf.transform.rotation = tf_rot
        self.broadcaster.sendTransform(self.tf)

    # def listen_tf(self, child, parent):
    #     tfBuffer = tf2_ros.Buffer()
    #     listener = tf2_ros.TransformListener(tfBuffer)
    #     rate = rospy.Rate(10.0)
    #     rate.sleep()

    #     try:
    #         trans = tfBuffer.lookup_transform(child, parent, rospy.Time())
    #         return trans
    #     except (
    #         tf2_ros.LookupException,
    #         tf2_ros.ConnectivityException,
    #         tf2_ros.ExtrapolationException,
    #     ) as err:
    #         rospy.loginfo("tf listen error%s" % err)
    #         return err
        
def generate_spiral_waypoints(num_points):
    waypoints = []
    radius = 0.3  # 初期半径
    theta = 0.0  # 初期角度
    theta_increment = 2 * np.pi / num_points  # 角度の増分
    scale = np.linspace(0, 1, num_points)
    for num in range(num_points):
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        z = scale[num]  # 高さ
        waypoint = [0, 0, 0, 0, 0, 0, 0]
        waypoint[0]= x
        waypoint[1] = y
        waypoint[2] = z
        waypoint[3] = 0.0
        waypoint[4] = 0.0
        waypoint[5] = 0.0
        waypoint[6] = 1.0
        waypoints.append(waypoint)

        radius += scale[1]  # 半径を増加させて螺旋状にする
        theta += theta_increment
        # print(z)
    # print(waypoints)
    return waypoints


def main(args=None):
    rclpy.init(args=args)
    waypoints = generate_spiral_waypoints(10)
    broadcaster = TFPublisher(waypoints)
    rclpy.spin(broadcaster)
    broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()