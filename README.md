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

- You can false use_fake_hardware option when you use real robot.
## トラブルシューティング
- RVizが真っ暗で何も表示されない
    - https://github.com/ros2/rviz/issues/948
    - `add-apt-repository ppa:kisak/kisak-mesa`して`apt update`と`apt upgrade`で解決
- moveitでexecuteしているのにUR5eが動かない
    - ros2 controlでセグフォがでている...
    - humble branchのros2_controlをunderlayに追加してビルドして通った
