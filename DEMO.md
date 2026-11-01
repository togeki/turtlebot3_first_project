# 最小動作確認（2026-11-01）

UTM / Ubuntu 24.04.3 / ROS 2 Jazzy / Gazebo Sim 8.10.0 / Burger でビルドを確認した。`simple_avoid` は実際の `/scan` を受信し、TwistStamped で速度指令を送る短時間の動作確認を行った。四角形全周走行や Nav2 ナビゲーションは今回未検証。

README 本文の Humble / Gazebo Classic 手順は旧環境向けの学習記録。Jazzy と混用しないこと。

## ビルド

リポジトリルートで：

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select tb3_first_py
```

## 実行

まずシミュレータを起動する。UTM での Ogre2 クラッシュを回避するため、関連プロジェクト [turtlebot3_llm_semantic_navigation](https://github.com/togeki/turtlebot3_llm_semantic_navigation) の整理済み `scripts/sim.sh` を使う。手順は同リポジトリの README を参照。シミュレータを二重起動しないこと。

別端末、**本リポジトリルート**で：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run tb3_first_py simple_avoid --ros-args -p use_sim_time:=true
```

Ctrl+C で終了する。同時に他の `/cmd_vel` 発行ノードを動かさない。`square_move` も同じ型に対応させたが、今回のコース全周動作は未検証。

旧環境の Twist bridge を使う場合は `-p stamped_cmd_vel:=false` を追加する。

このコードはレーザー値を使う規則制御で、AI モデルを呼ばない。単独で AI 課題の要件を満たすとは主張しない。センサ更新停止への対応など実機用の検証もない。

## 保存済み地図

同じ world・地図が適合する環境で Nav2 を試す場合、本リポジトリルートから：

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true map:="$(pwd)/map/turtlebot3_world_map.yaml"
```

この Nav2 経路は今回未検証。独自 launch は含まない。存在しない `avoid_move` のエントリを除去し、`sensor_msgs` 依存を追加した。現在の実行入口は `square_move` と `simple_avoid`。
