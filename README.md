## Powder Grinding Package for ROS2
## 環境
- ROS2 jazzy
  - ros2 hubmleでもいいんだけど、MoveIt2のpython実装がhumbleまで対応しないようなのでjazzyを採用
- Ubuntu 24.04

## Quick Start
Launch UR demo
```
ros2 launch grinding_robot_bringup ur5e_bringup.launch.xml 
```
Test scaled_joint_trajectory_controller
```
ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
```

## トラブルシューティング
**ビルド中に固まる**
- WSL2を使っていたり、性能低いPCだとビルド中に固まることがある
  - その場合は、ビルドの並列化数を下げると解決することがある
- colcon buildの引数で並列化数を明示的に設定する
  - `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --cmake-clean-cache`
  - `--parallel-workers 8`がビルドの並列数
  - デフォルトだとCPUのコア数に設定されているが、WSLを使っていたり他にリソースがさかれている場合はジョブがコンフリクトする可能性があるので、下げたほうが良い
  - 1だと最も安全(ビルドの時間が非常に長くなるけど)

## 開発ログ
2025/2/24
- 粉砕用のURDFを追加、モデル表示とMoveItの確認までできた
- 次にMotionの作成と確認
  - moveit_pyは以前のCommanderと比べて面倒そうなので、調べつつ
  - https://hara-jp.com/_default/ja/Topics/moveit_py%E3%82%92%E4%BD%BF%E3%81%86.html
2024/12/23
- これまでROS2のコンポーネント指向を理解せずに書いていたので、コンポーネント指向をだいぶ理解しました
- moveti planning sceneはコンポーネント指向で書き直し、rosparamを呼んでテーブルと乳鉢の追加まで書いて、bringupに追加しました
  - なぜか.pyのlaunchでパラメータのyamlが読めずはまっていますが、XMLのlaunchは動いたのでよしとします
- 次はgrinding motionを実装していきます
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
