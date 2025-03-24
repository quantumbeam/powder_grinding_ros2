import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


from tf_transformations import quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import Quaternion, Pose
from builtin_interfaces.msg import Duration
import numpy as np

def main():
    rclpy.init()
    print("Hello")
    point=Pose()
    point.position.x=1.0
    point.position.y=1.0
    point.position.z=1.0
    print(point)
    marker=Marker()
    marker.header.frame_id =  "map"
    marker.header.stamp = rclpy.clock.Clock().now().to_msg()
    marker.ns = "point"
    marker.id=1
    marker.action = Marker.ADD
    marker.pose.position = point.position
    marker.pose.orientation = point.orientation
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.type = Marker.SPHERE
    marker.lifetime = Duration(sec=100, nanosec=0)
    print(marker)

    # publisher wo tukuru

    node=Node("test_point")
    pub=node.create_publisher(Marker, "test_point", 10)
    pub.publish(marker)
    print("Published!")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
