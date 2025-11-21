🐢 TurtleBot3 First Project

ROS2 TurtleBot3 Virtual Project (Environment Setup • Basic Control • SLAM • Navigation)

🌱 A small ROS2 learning project using a virtual TurtleBot3 robot.
🌱 仮想 TurtleBot3 を使った ROS2 入門プロジェクト。

⸻

0. Project Overview / プロジェクト概要

English

This repository documents my learning process with ROS2 Humble + TurtleBot3 (Burger), including:
	•	Running TurtleBot3 in a virtual Gazebo simulation
	•	Teleoperating the robot with keyboard commands
	•	Performing 2D mapping using SLAM
	•	Saving maps and (later) testing navigation & path planning

All experiments are performed on Ubuntu 22.04 + ROS2 Humble, without using any real robot hardware.

日本語

このリポジトリは、ROS2 Humble + TurtleBot3（Burger） を使った学習記録です。
以下のステップをまとめています：
	•	Gazebo 上で TurtleBot3 をシミュレーション実行
	•	キーボード操作による移動
	•	SLAM による 2D マッピング
	•	地図保存および（今後追加予定の）ナビゲーションと経路計画

環境は Ubuntu 22.04 + ROS2 Humble を使用し、実機ロボットは使用していません。

⸻

Part 1. Environment Setup / 環境構築（ROS2 + TurtleBot3）

This section describes how to set up ROS2 Humble and TurtleBot3 on Ubuntu 22.04.
このパートでは、Ubuntu 22.04 上に ROS2 Humble と TurtleBot3 をセットアップします。

⸻

1-1. Install ROS2 Humble Desktop
