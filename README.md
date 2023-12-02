## ROS2 enviroments
**ROS2 humble on docker
**
## PC内環境でのURデモコマンド
Launch UR model on Rviz
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.42 use_fake_hardware:=true launch_rviz:=true
```
Launch UR model and MoveIt
- You can choose "false" of  use_fake_hardware option when you use real robot.
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.42 use_fake_hardware:=true launch_rviz:=false
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```

Test scaled_joint_trajectory_controller
```
ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
```



## Default DDS
- humbleでデフォルトのfast RTPSよりもcyclone DDSが良いらしいので、切り替えて使っている
  - cyclone DDSのインストールと変更は以下
    - `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
    - `apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp`
  - Fast RTPSに戻す場合は以下
    - `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`


## トラブルシューティング
**2023/12/2時点では解決しています**
### on ubuntu
**RVizが真っ暗で何も表示されない**
    - https://github.com/ros2/rviz/issues/948
    - `add-apt-repository ppa:kisak/kisak-mesa`して`apt update`と`apt upgrade`で解決

**moveitでexecuteしているのにUR5eが動かない**
    - ros2 controlでセグフォがでている...
    - humble branchのros2_controlをunderlayに追加してビルドして通った
    - **root権限のターミナルでビルドするとエラー解決しなかったので注意**

**ビルド時に`SetuptoolsDeprecationWarning`がでる**
- setuptoolsのバージョンを下げればでなくなるよう
  - https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/
  - `pip install setuptools==58.2.0`

