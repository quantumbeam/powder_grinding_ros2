## ROS2 enviroments
**ROS2 jazzy on docker
**
## PC内環境でのURデモコマンド
Launch UR model on Rviz
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.58.42 use_fake_hardware:=true launch_rviz:=true
```
Launch UR model and MoveIt
- You can choose "false" of  use_fake_hardware option when you use real robot.
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.58.42 use_fake_hardware:=true launch_rviz:=false
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```

Test scaled_joint_trajectory_controller
```
ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
```
## Default DDSの変更
- デフォルトのfast RTPSよりもcyclone DDSが良いらしい
  - https://research.reazon.jp/blog/2023-01-15-DDS-performance.html
- 切り替えたDDSが使えるのかはパッケージ依存なので、確認してから使ってください
- cyclone DDSのインストールと変更は以下
  - `apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp`
  - `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- Fast RTPSに戻す場合は以下
  - `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`

## メモ
2024/9/29
- joint trajectory controllerとcontroller managerのrqtを追加しました
- moveti planning sceneでboxを追加するところまで書きました
  - 続きはrosparamを読んで乳鉢等を追加するように変更します
- grinding motionの実装はまだです
2024/05/04
- URのドライバもMoveIt2もバイナリインストールで動くようになったので、Dockerfileを修正しました
  - ビルド負荷が減ってかなり楽
2023/12/03
- grinding_descriptionsとgrinding_motion_routinesを移植している途中です
  - まだ、どちらも動きません
  - とりあえず、descriptionsでmoveti planning scene表示して、motion_routinesでgrinding motionできるところまで確認するのが目標です
  - 現状だと、descriptionsでparameterをdeclareしているところでエラーがでているので、それを解決する必要があります

## トラブルシューティング
**ビルド中に固まる**
- WSL2を使っていたり、性能低いPCだとビルド中に固まることがある
  - その場合は、ビルドの並列化数を下げると解決することがある
- colcon buildの引数で並列化数を明示的に設定する
  - `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --cmake-clean-cache`
  - `--parallel-workers 8`がビルドの並列数
  - デフォルトだとCPUのコア数に設定されているが、WSLを使っていたり他にリソースがさかれている場合はジョブがコンフリクトする可能性があるので、下げたほうが良い
  - 1だと最も安全(ビルドの時間が非常に長くなるけど)

**URの実機がcyclonDDSで動かない**
  - 2023/12/02時点では、cyclonDDSで動かないので、実機ではfastRTPSに戻しています
