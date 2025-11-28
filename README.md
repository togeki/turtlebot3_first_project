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
- Nav2 を用いたナビゲーションおよび経路計画（※今後追加予定）

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

Gazebo 上に TurtleBot3 が生成されます。

## 3-2. SLAM ノード起動（Cartographer）

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

Rviz2 にリアルタイムで地図が描かれます。

## 3-3. キーボードでロボットを動かす

別ターミナルで teleop：

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

ロボットを動かすことで SLAM の地図が広がっていきます。

## 3-3. 地図を保存する

```bash
ros2 run nav2_map_server map_saver_cli -f ~/turtlebot3_world_map
```

---

#Part 4. Navigation2 による自律移動

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

## 1-3. 

2D Pose Estimate → ロボットの位置と向きを指定

Nav2 Goal → 目標位置と向きを指定
