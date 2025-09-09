#!/usr/bin/env python3
import rclpy
from visualization_msgs.msg import Marker # For Marker.SPHERE in main
from geometry_msgs.msg import Pose
import numpy as np

# grinding_motion_routines パッケージの DisplayMarker をインポート
from grinding_motion_routines.marker_publisher import MarkerPublisher


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
        # Orientation will be default (0,0,0,1 or 0,0,0,0)
        waypoints.append(waypoint)

        radius += scale[1]  # 半径を増加させて螺旋状にする
        theta += theta_increment
    return waypoints


def main(args=None):
    rclpy.init(args=args)
    waypoints = generate_spiral_waypoints(100)
    # DisplayMarkerノードは内部で "marker_display" という名前で初期化されます
    marker_display = MarkerPublisher()
    
    # display_waypoints は内部で wait_for_connection を呼び出し、
    # RVizなどのサブスクライバが接続するのを待ってからマーカーをパブリッシュします。
    marker_display.display_waypoints(waypoints, scale=0.01, type=Marker.SPHERE)

    # marker_display.display_waypoints(waypoints) # 元のコメント
    
    # marker_display.create_timer(1.0, marker_display.display_waypoints(waypoints))  # 元のコメント
    rclpy.spin(marker_display) # ノードをスピンさせてアクティブに保ちます
    
    marker_display.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
