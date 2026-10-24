# 验证记录（2026-10-18）

- 在 Ubuntu 22.04 / ROS 2 Humble 中，从本目录源码编译 SDK、livox_ros_driver2、fast_lio、ros2_to_stm32，全部完成。
- 三个 ROS 包的 prefix 均位于本目录 lidar_ws/install；动态链接的 Livox SDK 位于本目录 .local/lib。
- FAST-LIO 参数数值与指定 go2_fastlio_localization/config/fast_lio_mid360_real.yaml 的当前快照完全一致。
- Python / Shell 语法检查通过，旧控制方法的 AST 与原版一致，仅适配配置文件路径。
- 只启动 FAST-LIO，回放 horse_localization_20261012_171609 的 /livox/lidar 和 /livox/imu，使用隔离 ROS_DOMAIN_ID=183。
- 最终收到 46 条里程计和 46 条当前点云；收到的累计地图从 145694 点增长至 167809 点。
- 位姿数值有限；Odometry 帧为 odom -> livox_frame，点云和地图帧为 odom。
- /map_save 返回 success=false、Map save disabled.，退出后未新增 PCD 文件。
- replay_result.json 保存机器可读结果。接收计数不是输出频率验收，不能据此推算实机频率。
- 首次较旧静止录包回放的保存服务等待 5 秒超时；换用较新录包并扩大服务等待至 15 秒后通过。未将首次失败认定为算法故障或宣称其根因已确定。
- 未连接物理雷达、未打开 RViz 窗口、未启动串口或运动控制。当前 WSL 完整业务依赖仍缺 tf_transformations；传感器链编译与回放不依赖它。

重新回放：

```bash
source run/environment.sh
python3 validation/replay_check.py /你的录包目录 --duration 35
```

脚本仅启动本地 FAST-LIO 与录包播放器，不启动真实驱动和控制器。
