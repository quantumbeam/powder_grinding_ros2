#!/bin/bash

# ---------------------------------------------------
# 1. システムの更新と必要なツールのインストール
# ---------------------------------------------------
echo "### 1. Updating system and installing necessary tools ###"
sudo apt update
sudo apt upgrade -y
sudo apt install -y curl software-properties-common

# ---------------------------------------------------
# 2. ロケールの設定 (UTF-8 サポートの確認と設定)
# ---------------------------------------------------
echo "### 2. Setting locale to support UTF-8 ###"
# 現在のロケールを確認
locale
# locales パッケージがインストールされていない場合はインストール
sudo apt install -y locales
# en_US.UTF-8 ロケールを生成
sudo locale-gen en_US en_US.UTF-8
# システムのデフォルトロケールを設定
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# 現在のシェルセッションにロケールをエクスポート
export LANG=en_US.UTF-8
# 設定が適用されたことを確認
echo "Verifying locale settings:"
locale

# ---------------------------------------------------
# 3. ROS 2 ソースのセットアップ
# ---------------------------------------------------
echo "### 3. Setting up ROS 2 sources ###"
# Ubuntu Universe リポジトリを有効にする
echo "Enabling Ubuntu Universe repository..."
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

# ROS 2 GPG キーを追加
echo "Adding ROS 2 GPG key..."
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS 2 リポジトリをソースリストに追加
echo "Adding ROS 2 repository to sources list..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ---------------------------------------------------
# 4. ROS 2 パッケージのインストール
# ---------------------------------------------------
echo "### 4. Installing ROS 2 packages ###"
# リポジトリキャッシュを更新
sudo apt update

# システムが最新であることを確認 (重要: systemd および udev 関連パッケージの更新)
echo "Ensuring system is up to date before installing ROS 2 packages (important for Ubuntu 22.04 systemd/udev updates)..."
sudo apt upgrade -y

# ROS 2 Humble Desktop Install (推奨: ROS, RViz, デモ, チュートリアルを含む)
echo "Installing ros-humble-desktop..."
sudo apt install -y ros-humble-desktop

# 開発ツールのインストール (ROS パッケージのビルドに必要なコンパイラなど)
echo "Installing ros-dev-tools..."
sudo apt install -y ros-dev-tools

# rosdep の初期化と更新
echo "Initializing and updating rosdep..."
sudo rosdep init --rosdistro humble # ROS 2 では --rosdistro を指定することが推奨されます
rosdep update

# ---------------------------------------------------
# 5. 追加のAPTパッケージのインストール
# ---------------------------------------------------
echo "### 5. Installing additional common APT packages ###"
./docker/install_packages.sh

# ---------------------------------------------------
# 6. 環境設定
# ---------------------------------------------------
echo "### 6. Setting up environment variables ###"
# ROS 2 セットアップスクリプトを ~/.bashrc に追加
sh -c 'echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc'
sh -c 'echo "export ROS_DISTRO=humble" >> ~/.bashrc'

# colcon_cd (便利なディレクトリ移動ツール) の設定 (ros-humble-desktop に含まれる場合がありますが、明示的に追加)
# sh -c 'echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc'
# sh -c 'echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc' # 必要であればコメント解除

# ROS 2 ワークスペースの設定例 (既存のROS 1ワークスペースは直接使用できません)
# 新しいROS 2ワークスペースを作成する例
# mkdir -p ~/ros2_ws/src
# sh -c 'echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc' # ワークスペースビルド後に必要

# ビルドエイリアス (colcon build を使用)
sh -c "echo \"alias b='cd ~/ros2_ws; colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release; source install/setup.bash'\" >> ~/.bashrc"

# Copy build ros workspace script
cp ./docker/INITIAL_SETUP_ROS_ENVIROMENTS.sh ~/ros2_ws/
cp ./docker/BUILD_ROS_WORKSPACE.sh ~/ros2_ws/


# ---------------------------------------------------
# 7. Python 仮想環境の作成
# ---------------------------------------------------
echo "### 7. Creating Python Virtual Environment ###"
# 仮想環境を作成するディレクトリ
VENV_DIR="$HOME/ros2_ws/venv"
ROS2_WS_DIR="$HOME/ros2_ws"

# ROS 2 ワークスペースディレクトリが存在しない場合は作成
mkdir -p "$ROS2_WS_DIR"

# Python 仮想環境を作成
echo "Creating virtual environment at $VENV_DIR..."
python3 -m venv "$VENV_DIR"

# .bashrc に仮想環境のアクティベーションと設定を追加
sh -c 'echo "" >> ~/.bashrc' # 空行を追加して区切りを明確にする
sh -c 'echo "# Python Virtual Environment for ROS 2" >> ~/.bashrc'
sh -c "echo \"source \\\"$VENV_DIR/bin/activate\\\"\" >> ~/.bashrc"
sh -c 'echo "export VIRTUAL_ENV=\"$VENV_DIR\"" >> ~/.bashrc'
sh -c 'echo "export PATH=\"$VIRTUAL_ENV/bin:$PATH\"" >> ~/.bashrc'
sh -c 'echo "" >> ~/.bashrc'

# colcon が venv ディレクトリを検索しないように COLCON_IGNORE ファイルを作成
echo "Creating COLCON_IGNORE in $VENV_DIR to prevent colcon from searching it..."
touch "$VENV_DIR/COLCON_IGNORE"

# 現在のシェルセッションに仮想環境をアクティベート
echo "Activating virtual environment in current session..."
source "$VENV_DIR/bin/activate"


# 設定を現在のシェルに反映
echo "Sourcing ~/.bashrc to apply changes..."
source ~/.bashrc


echo "---------------------------------------------------"
echo "ROS 2 Humble Installation Complete!"
echo "Please restart your terminal or run 'source ~/.bashrc' to ensure all changes are applied."
echo "---------------------------------------------------"