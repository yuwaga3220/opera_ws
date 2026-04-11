# opera_ws

研究用のROS 2（colcon）ワークスペースです。Unity 連携や経路追従・オーバーレイ表示などのパッケージを置いています。

## 含まれる主なパッケージ（`src/`）

| パッケージ | 概要 |
|-----------|------|
| `pure_pursuit_ugo` | Pure Pursuit 系の制御・カメラ画像へのオーバーレイ（アーム簡易キネマティクス、土のう画像の投影など） |
| `com3_ros` | `com3_msgs` など |
| `ROS-TCP-Endpoint` | ROS と Unity などの TCP ブリッジ |