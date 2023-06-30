import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


from tf_transformations import quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import Quaternion, Pose
from builtin_interfaces.msg import Duration
import numpy as np


class MarkerDisplay(Node):
    def __init__(self, marker_publisher_name):
        super().__init__("marker_display")
        self.publisher = self.create_publisher(MarkerArray, marker_publisher_name, 10)
        self.rate = self.create_rate(10)
        self.index = 0

    def display_waypoints(self, waypoints, scale=0.02, type=None):
        marker_array = MarkerArray()
        if type is None:
            type = Marker().SPHERE
        for index, points in enumerate(waypoints):
            marker = Marker()

            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            self.index += 1
            marker.id = self.index
            print(marker.id)
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
            marker.lifetime = Duration()  # existing marker for ever
            marker_array.markers.append(marker)

        self.publisher.publish(marker_array)


def generate_spiral_waypoints(self, num_points):
    waypoints = []
    radius = 0.3  # 初期半径
    theta = 0.0  # 初期角度
    theta_increment = 2 * np.pi / num_points  # 角度の増分

    for num in range(num_points):
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        z = num * 0.01  # 高さ
        waypoint = Pose()
        waypoint.position.x = x
        waypoint.position.y = y
        waypoint.position.z = z
        waypoints.append(waypoint)

        radius += 0.1  # 半径を増加させて螺旋状にする
        theta += theta_increment
        print(x, y, z)
    return waypoints


def main(args=None):
    rclpy.init(args=args)
    marker_display = MarkerDisplay("marker_publisher")
    waypoints = generate_spiral_waypoints(marker_display, 10)
    marker_display.display_waypoints(
        waypoints
    )  # replace 'waypoints' with your waypoints data
    rclpy.spin(marker_display)
    marker_display.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
