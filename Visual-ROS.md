## ROS for light user
- ROSは現状だとロボットエンジニア or ロボット研究者しか使えない
- 色々なロボットに使えたり、分散型のシステムになっている点は使い勝手が良いと思うので、もっと多くの人が使いやすい形にするにはどうしたらよいか？
- 現状、ROS(特にROS2)はフレキシビリティが高すぎて、何でもできるけど設定や記述が非常に面倒。慣れるまで時間がかかる
- とりあえず全体俯瞰のためにROSのシステムをVisualで示すようにして(これはRVizという意味ではなくノードのGUIのイメージ)、ノードの作成もGUIでできるようにすると、最低限必要な機能に絞れるはず
- 案としては、NodeRedのようなGUIでノードを作成し、プログラムの作成もblocklyのようなGUIでできるようにすると、ROSを使いやすくできるのではないか？

## NodeRed for ROS2
- すでにあった
- https://www.youtube.com/watch?v=RmDEFXi7nJs
- https://github.com/eProsima/node-red-ros2-plugin

## NodeRedのインストール
```bash
# Node.js（v16以上が推奨）をインストール
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt install -y nodejs

# Node-REDをインストール
sudo npm install -g --unsafe-perm node-red

```