# 实车参数

调参入口是 `robot.yaml`。导航、预设地图、传感器建图和旧串口的常用入口均加载这份配置。按节点名查找参数。

| 内容 | YAML 键 |
| --- | --- |
| IMU 到底盘外参、里程计阈值 | `nav_adapter` |
| 速度限幅、消息和 TF 看门狗 | `nav_cmd_gate` |
| DWB、到点容差、地图和代价图 | `controller_server`、`planner_server`、`local_costmap`、`global_costmap` |
| 雷达和 FAST-LIO | `mid360_real_driver`、`laser_mapping` |
| 点云切片和 GICP | `pointcloud_to_laserscan`、`gicp_localizer` |
| 旧动作串口、任务超时 | `motor_serial`、`task_to_do`、`nav_goal_client` |
| 二维制图默认值、采点净空默认值 | map_projection、map_task_editor |

在导航启动脚本内用 `--params-file /绝对路径/试验.yaml` 覆盖某些 ROS 参数。直接调用 ROS launch 时写 `params_file:=/绝对路径/试验.yaml`。修改后重启，用 `ros2 param get /nav_cmd_gate max_forward_speed` 等命令核对值。若安装目录不是源码符号链接，更新配置后需要同步安装。用 `bash run/build_ros2.sh --gicp` 安装或更新。

MID-360 的 IP 和端口由相邻的 `mid360_real.json` 提供。点云和地图数据、启动区平面位姿和 Nav2 目标保存在 `maps/<地图名>/`。启动台高度仍由 `--imu-z` 指定。

旧的四个 YAML 文件名保留为指向 robot.yaml 的符号链接，便于现有安装目录升级。所有实车调参请直接修改 robot.yaml。
