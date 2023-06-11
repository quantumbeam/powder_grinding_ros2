## 環境
- ROS2 humble on docker

## dockerの立ち上げ
- `docker run -p 6080:80 --shm-size=512m --security-opt seccomp=unconfined tiryoh/ros2-desktop-vnc:humble`
## コマンド
- 
- ` vcs import src < src/underlay.rosinstall`
- `rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y`
- `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`
    - clean cache option `--cmake-clean-cache`
## PC内環境でのURデモコマンド
demo1
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.42 use_fake_hardware:=true launch_rviz:=true
ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
```

moveit demo
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.42 use_fake_hardware:=true launch_rviz:=false
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```

- You can choose "false" of  use_fake_hardware option when you use real robot.

## コメント
**moveit2 チュートリアル**
- 全然だめ、チュートリアルいれて環境構築したらmoveitでまともにUR動かすことすらできなくなって、トラブル多発したので一旦諦めた(.rosinstallにコメントアウトして一応残っている)
  - チュートリアルではmoveitのバージョン指定があるので、その影響動かなかったのだと思われる

## トラブルシューティング
### on ubuntu
**RVizが真っ暗で何も表示されない**
    - https://github.com/ros2/rviz/issues/948
    - `add-apt-repository ppa:kisak/kisak-mesa`して`apt update`と`apt upgrade`で解決

**moveitでexecuteしているのにUR5eが動かない**
    - ros2 controlでセグフォがでている...
    - humble branchのros2_controlをunderlayに追加してビルドして通った
    - **root権限のターミナルでビルドするとエラー解決しなかったので注意**


### on windows
```
[publisher_joint_trajectory_controller-1] 2023-06-10 10:50:31.731 [RTPS_TRANSPORT_SHM Error] Failed to create segment 16a2460f75a95f65: Too many levels of symbolic links -> Function compute_per_allocation_extra_size
```
- どうやらDDSのFastRTPTでトラブルが起こっている模様
  - cyclone DDSが割と動作ましらしいし、切り替えてみてそのままエラー解決
    - `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
    - `apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp`
  - ただ、humbleのデフォルトはFastRTPSだし、ubuntuでcycloneDDSで実機動かそうと思ったらros2_controlの一部のパッケージが未対応で事故ったので、注意