import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


from tf_transformations import quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import Quaternion, Pose
from builtin_interfaces.msg import Duration
import numpy as np
import time

class MarkerDisplay(Node):
    def __init__(self, marker_publisher_name, waypoints):
        super().__init__("marker_display")
        self.publisher = self.create_publisher(MarkerArray, marker_publisher_name, 1)
        # self.rate = self.create_rate(10)  
        self.index = 0
        self.waypoints = waypoints
        # self.timer = self.create_timer(1.0, self.executer)  # 1秒ごとに送信
    def wait_for_connection(self):
        self.get_logger().info('Waiting for publisher to connect...')
        while(self.publisher.get_subscription_count() == 0):
        #   rclpy.spin_once(self)
          time.sleep(1)
          counter=self.publisher.get_subscription_count()
          print(counter)
        self.get_logger().info('Publisher connected!')


        
    def display_waypoints(self, waypoints, scale=0.02, type=None):
        marker_array = MarkerArray()

        if type is None:
            type = Marker().SPHERE
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
            # print(marker.id)
            marker.action = Marker.ADD

            marker.pose.position = points.position
            marker.pose.orientation = points.orientation

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
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
        # print("Published!")
    def executer(self):
        self.display_waypoints(self.waypoints)
        # self.rate.sleep()

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
        waypoint = Pose()
        waypoint.position.x = x
        waypoint.position.y = y
        waypoint.position.z = z
        waypoints.append(waypoint)

        radius += scale[1]  # 半径を増加させて螺旋状にする
        theta += theta_increment
        # print(z)
    return waypoints


def main(args=None):
    rclpy.init()
    waypoints = generate_spiral_waypoints(100)
    marker_display = MarkerDisplay("marker_publisher", waypoints)
    marker_display.display_waypoints(waypoints)

    # marker_display.display_waypoints(waypoints)
    
    # marker_display.create_timer(1.0, marker_display.display_waypoints(waypoints))  # replace 'waypoints' with your waypoints data
    rclpy.spin(marker_display)
    marker_display.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    
