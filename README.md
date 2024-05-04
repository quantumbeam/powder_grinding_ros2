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
    - `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
    - `apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp`
  - Fast RTPSに戻す場合は以下
    - `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`

## メモ
2023/12/03時点でのメモ
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
  - 2023/12/02時点では、cyclonDDSで動かないので、fastRTPSに戻しています

**URをMoveItで動かそうとコードを書いてみたけど`Could not find parameter robot_description_semantic and did not receive robot_description_semantic `のエラーがでる**
- ココらへんで議論がある
  - https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/457
  - https://forum.universal-robots.com/t/move-ur-with-a-moveit2-script/24497
- 多分、PlanningSceneなどを実行したノードのパラメータにrobot_description_semanticとかがないので、読めないエラーがでている
  - 明示的に指定するか、UR Driverの修正を待つ(もしくは修正する)か
- 明示的に指定する場合、このlaunchの書き方を参考にするとよい(上のissueで議論する中で作ったpkg)
  - https://github.com/LucaBross/simple_moveit2_universal_robots_movement/tree/main

**以下は2023/12/2時点では解決しています**
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

