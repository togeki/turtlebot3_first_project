# TurtleBot3 First Project  
ROS2 TurtleBot3 仮想プロジェクト（環境構築・基本操作・SLAM・ナビゲーション）

> 仮想 TurtleBot3（Burger）を使った ROS2 Humble 入門プロジェクトです。  
> Gazebo 上で TurtleBot3 を操作し、SLAM による地図生成、地図保存、ナビゲーション・経路計画までの流れをまとめています。

---

## 0. プロジェクト概要 / Project Overview

本リポジトリでは、ROS2 Humble + TurtleBot3（Burger）を使った学習記録をまとめています。  
主に以下を扱います：

- Gazebo 上で TurtleBot3 をシミュレーション実行  
- キーボード操作によるロボットの移動  
- SLAM による 2D マッピング（地図生成）
- 地図の保存  
- Nav2 を用いたナビゲーションおよび経路計画

**使用環境：**

- Ubuntu 22.04 LTS  
- ROS2 Humble  
- Gazebo + Rviz2  
- TurtleBot3 Burger（仮想ロボット）

---

# Part 1. 環境構築（ROS2 + TurtleBot3）

ここでは、Ubuntu 22.04 上に ROS2 Humble と TurtleBot3 をセットアップします。

---

## 1-1. ROS2 Humble のインストール

```bash
sudo apt update
sudo apt install ros-humble-desktop -y
```

## 1-2. ROS2 アップデート・環境変数設定

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 1-3. TurtleBot3 パッケージのインストール

```bash
sudo apt install ros-humble-turtlebot3* -y
```

TurtleBot3 モデル設定（Burger を使用）：

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```
---

# Part 2. キーボード操作（Teleop）

TurtleBot3 をキーボードで操作します。

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

w / a / s / d / x キーでロボットが前後左右に移動します。

---

# Part 3. SLAM による地図生成（SLAM with Cartographer）

Gazebo 上で TurtleBot3 を動かしながら SLAM を実行し、地図を作成します。

---

## 3-1. Gazebo シミュレーション起動

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
<img width="624" height="649" alt="image" src="https://github.com/user-attachments/assets/bafc1638-4471-4bec-8074-c8683c2c3de6" />


Gazebo 上に TurtleBot3 が生成されます。

## 3-2. SLAM ノード起動（Cartographer）

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```
<img width="554" height="684" alt="image" src="https://github.com/user-attachments/assets/9d5e6443-9443-4dee-a76e-548012529d93" />


Rviz2 にリアルタイムで地図が描かれます。

## 3-3. キーボードでロボットを動かす

別ターミナルで teleop：

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

ロボットを動かすことで SLAM の地図が広がっていきます。

## 3-4. 地図を保存する

```bash
ros2 run nav2_map_server map_saver_cli -f ~/turtlebot3_world_map
```
使用した地図ファイルは ./map 以下に配置しています。

---

# Part 4. Navigation2 による自律移動

## 1-1. Gazebo シミュレーション起動

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## 1-2. Navigation2起動

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  map:=$HOME/turtlebot3_world_map.yaml
```

## 1-3. Rviz2 で目標位置を指定

2D Pose Estimate → ロボットの位置と向きを指定

Nav2 Goal → 目標位置と向きを指定

## 1-4. 経路計画と自律移動（Path Planning & Autonomous Navigation）

Rviz2 上で「2D Pose Estimate」で自己位置を与え，
「Nav2 Goal」で目的地を設定すると，
ロボットはグローバルプランナーとローカルプランナーを用いて
自律的に経路を生成し，障害物を避けながら移動します。
<img width="835" height="723" alt="image" src="https://github.com/user-attachments/assets/395a873a-fe06-4202-bf32-7ea65824e154" />

本プロジェクトにより，ROS2 を用いた自律移動ロボット開発の基礎プロセス
（環境構築 → センサ情報取得 → SLAM → 地図生成 → ローカライズ → 経路計画 → 自律移動）を一通り習得しました。

---

# Part 5. Python ノードによる自律的な障害物回避

本フェーズでは，SLAM や Nav2 を使用せず，LaserScan `/scan` トピックのみを用いた  
**ローカル意思決定（local decision making）による自律走行**を実装した。

## 1-1. ROS2 ワークスペースの作成

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

## 1-2. Python パッケージの作成

```bash
ros2 pkg create --build-type ament_python tb3_first_py --dependencies rclpy geometry_msgs sensor_msgs
```

生成される構造:

ros2_ws/src/tb3_first_py/
 ├─ package.xml
 ├─ setup.py
 └─ tb3_first_py/
     └─ __init__.py

## 1-3. 自律移動ノードの追加:

setup.py
```code
entry_points={
    'console_scripts': [
        'maze_left = tb3_first_py.maze_left:main',
    ],
}
```
