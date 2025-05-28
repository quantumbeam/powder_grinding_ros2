#!/usr/bin/env python3
import rclpy
import numpy as np
# display_tf.py から DisplayTF クラスをインポート
from grinding_motion_routines.display_tf import DisplayTF


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
        # [x, y, z, qx, qy, qz, qw] の形式でリストを作成
        waypoint = [0.0] * 7
        waypoint[0]= x
        waypoint[1] = y
        waypoint[2] = z
        waypoint[3] = 0.0 # qx
        waypoint[4] = 0.0 # qy
        waypoint[5] = 0.0 # qz
        waypoint[6] = 1.0 # qw (単位クォータニオン、回転なし)
        waypoints.append(waypoint)

        radius += scale[1]  # 半径を増加させて螺旋状にする
        theta += theta_increment
    return waypoints


def main(args=None):
    rclpy.init(args=args)

    # DisplayTFノードのインスタンスを作成
    # DisplayTFは内部で "tf_publisher_node" という名前のノードを作成する
    tf_broadcaster_node = DisplayTF(parent_link="world", child_link="spiral_point_")
                                    # parent_link と child_link のプレフィックスを指定

    waypoints_data = generate_spiral_waypoints(20) # ポイント数を調整可能

    # TFを一度だけ発行する
    # broadcast_tf_with_waypoints は、指定されたすべてのウェイポイントに対してTFを発行します。
    # TFは通常、最新の状態を継続的に発行するものですが、このデモでは
    # 各ウェイポイントに対応する静的なTFフレームを一度発行し、
    # RVizなどで表示され続けることを想定しています。
    tf_broadcaster_node.broadcast_tf_with_waypoints(waypoints_data)
    tf_broadcaster_node.get_logger().info(
        f"Broadcasted {len(waypoints_data)} TFs with child prefix 'spiral_point_'. Spinning to keep them visible."
    )
    tf_broadcaster_node.get_logger().info(
        "In RViz, you might need to set a long 'TF Decay Time' in the TF display properties if the TFs disappear."
    )

    try:
        # ノードをスピンさせてTFが(RVizなどで)表示され続けるようにする
        # (実際にはsendTransformは上記の呼び出しで一度実行されるだけですが、
        #  ノードがアクティブである必要があります)
        rclpy.spin(tf_broadcaster_node)
    except KeyboardInterrupt:
        tf_broadcaster_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        # ノードを破棄
        tf_broadcaster_node.destroy_node()
        # rclpyをシャットダウン
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()