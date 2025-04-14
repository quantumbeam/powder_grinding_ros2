import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__("test_point")
        self.publisher = self.create_publisher(Marker, "test_point", 10)
        self.timer = self.create_timer(1.0, self.publish_marker)  # 1秒ごとに送信
        # qos_profile = QoSProfile(
        #     reliability=QoSReliabilityPolicy.RELIABLE,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # これが重要
        #     depth=1
        # )
        # self.executer=self.publish_marker()

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"  # フレームIDを適切に変更
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "point"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, 0)  # 回転なし
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # 透明度

        marker.lifetime = Duration(sec=10)  # 0なら無限に表示

        self.publisher.publish(marker)
        self.get_logger().info("Published marker")

def main():
    rclpy.init()
    node = MarkerPublisher()
    rclpy.spin(node)
    # node.publish_marker()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
