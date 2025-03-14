## Powder Grinding Package for ROS2

<!-- test kawasaki -->

## 対応ロボット
- 実装済み
  - UR5e (Universal Robot社)
- 今後実装予定
   - UR3e (Universal Robot社)
   - Cobotta (DENSOWAVE社)
   - Cobotta PRO 900 (DENSOWAVE社)
   - FR3 (FAIRINO 社)


## Quick Start
### PCとロボットとDocker環境のセットアップ
- [環境セットアップの資料](./env/docker/README_jp.md)を読んで環境セットアップし、終わったらこちらに戻ってきて以下の続きを実行してください。

### Dockerイメージの作成
```
cd ./env && ./BUILD-DOCKER-IMAGE.sh
```

### Dockerコンテナのビルド
- Dockerコンテナのビルド
   - `cd ./env && ./BUILD-DOCKER-IMAGE.sh`

### Dockerコンテナの立ち上げ
- ターミナル内でのDockerコンテナの立ち上げ
   - `cd ./env && ./RUN-DOCKER-CONTAINER.sh`
- Terminatorによる複数ターミナルの起動とDockerコンテナの立ち上げ
   - `cd ./env && ./LAUNCH-TERMINATOR-TERMINAL.sh`
      - 立ち上げられた複数ターミナルでは`RUN-DOCKER-CONTAINER.sh`が自動実行されている。

### Dockerコンテナ内でのROS環境のビルド
- 初回のみ実行
  - `./INITIAL_SETUP_ROS_ENVIROMENTS.sh`  
  -  以上のコマンドは`grinding_ws` のディレクトリ内で実行すること
- サードパーティ及び依存パッケージのアップデート+自作パッケージのビルド
  - `./BUILD_ROS_WORKSPACE.sh`
  -  以上のコマンドは`grinding_ws` のディレクトリ内で実行すること
- 自作パッケージのビルドのみ
  - `b` コマンドで実行できるようにエイリアスを組んである

## Demo
Launch UR5e
```
ros2 launch grinding_robot_bringup ur5e_bringup.launch.xml 
```

Test scaled_joint_trajectory_controller
```
ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
```

## 開発予定のパッケージ
- grinding_robot_bringup
  - ロボットの起動(モデル表示やMoveItの起動を含む)
  - 基本はこのパッケージでロボットを起動してモーションを実行する
- grinding_robot_description
  - ロボットのURDF、モデル表示テスト用のlaunchファイル
- grinding_moveit_config
  - 粉砕用のエンドエフェクタ等を設定したMoveItの設定ファイル
- grinding_scene_description
  - ロボットの周りの環境のメッシュファイルやMoveItのplanning sceneの設定を使いやすくするためのラッパーを提供
- grinding_robot_control
  - 既存のロボットアーム用コントローラーを使いやすくするためのラッパーを提供
  - 現状だとjoint_trajectory_controller、MoveItに対応
  - このパッケージはgrindingに限らず使えます
- grinding_motion_routines
  - grinding用のモーション生成(姿勢計算なのでROS非依存)と実行を提供
- grinding_force_torque_sensor
  - フォーストルクセンサーのデータを取得やフィルタリングの機能を提供
  - 対応センサは、UReシリーズ、leptrinoを予定
- gridning_calibration
  - 乳棒の長さ推定や、乳鉢の位置推定の機能を提供
  - grinding_force_torque_sensorとセットで使うことを想定

## Migration Progress 
- [ ] grinding_robot_bringup
  - [ ] UR5e/UR3e
  - [ ] Cobotta
- [ ] grinding_robot_description
  - [ ] UR5e/UR3e
  - [ ] Cobotta
- [ ] grinding_moveit_config
  - [ ] UR5e/UR3e
  - [ ] Cobotta
- [x] grinding_scene_description
- [ ] grinding_robot_control
  - [ ] JointTrajectoryController
  - [ ] MoveIt2
- [ ] grinding_motion_routines
  - [ ] GrindingMotion
  - [ ] GatheringMotion
- [ ] grinding_force_torque_sensor
  - [ ] WrenchFilter
- [ ] gridning_calibration
  - [ ] PestleLengthEstimation
  - [ ] MortarPositionEstimation

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
