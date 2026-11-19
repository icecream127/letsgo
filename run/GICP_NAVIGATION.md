# PCD 重定位与 Nav2

`run/navigation_gicp.sh` 启动 FAST-LIO、基于 small_gicp 的固定 PCD GICP、二维地图服务和 Nav2。项目现有 `run/navigation.sh` 默认使用 AMCL。两个入口各自启动完整的导航节点，同一 ROS 域中使用其中一个。

## 构建

在 Ubuntu 22.04 / ROS 2 Humble 中，从项目根目录运行：

```bash
cd /home/zxy/letsgo
bash run/build_ros2.sh --gicp
```

该入口下载并固定 CPU `small_gicp` v1.0.0（`fd29d8cf94cf05ed7ad21c81c27b65963110adb5`），启用 OpenMP，并构建 `go2_real_localization` 与 `go2_obstacle_interfaces`。首次构建需要 GitHub 网络和 PCL 开发包、`pcl_conversions`、`message_filters` 等 ROS 依赖。在新机可先按根 README 安装依赖，并以 `lidar_ws/src` 与已检出的 `.local/small_gicp` 为 rosdep 路径。定位包和消息包取自本机 `rc2026_deom` 工作树（基线提交 `77ae274a0f4f8e38086d9ebfc9e27a028f900647`），保留原许可。移除了未在 letsgo 使用的 `go2_fastlio_localization` 运行依赖。

## 使用保存的启动区

在地图编辑器确认正式启动区，点击“保存任务工程”，得到 `competition_tasks.json`。启动入口校验工程与当前二维地图，并使用生成记录中对应的 PCD。若地图更新，先核对启动区是否仍对应同一现场位置，再用当前编辑器保存工程。

测量并填写 `lidar_ws/src/ros2_to_stm32/config/nav2.yaml` 中的 `livox_to_base_xyz` 和 `livox_to_base_yaw`。前者是从 IMU 原点指向底盘参考点的向量，分量使用 IMU 本地轴（米），后者是底盘前向相对 IMU 的 yaw（弧度）。默认零值只是占位。`--imu-z` 是机器人在启动区时 IMU 在原始 PCD 坐标中的 z（米），需要根据 PCD 坐标与实际安装高度确定。存在已知倾斜时，用 `--imu-roll` 与 `--imu-pitch` 指定初猜（弧度）。入口将底盘的 `map` 平面位置反算成 IMU 的初始位置。

先加 `--check-only` 查看转换结果，确认后去掉该选项启动：

```bash
cd /home/zxy/letsgo
read -rp '输入已核对的 IMU 初始 z（米）: ' imu_z
bash run/navigation_gicp.sh competition \
  --project /mnt/c/Users/22512/Downloads/competition_tasks.json \
  --imu-z "$imu_z" --check-only
```

录包调试使用 `--no-sensors`，录包须提供原始 FAST-LIO 的 `/Odometry`、`/cloud_registered_body` 和 `odom → livox_frame`。录包提供 `/clock` 时加 `--use-sim-time`，可加 `--no-rviz` 关闭图形界面。ROS 域与 DDS 设置应与传感器或回放一致。人工重置在 GICP 模式使用 `/gicp/initialpose`，内容是 `map` 中的 **IMU** 位姿。常规启动由任务工程提供初猜。

## 检查匹配和规划

在相同 ROS 环境下观察 `/gicp/input_status` 的同步计数、`/gicp/status` 的匹配状态和 `/gicp/evidence` 的接受结果。使用 `ros2 run tf2_ros tf2_echo map base_footprint` 检查底盘位姿，并在 RViz 对照地图和对齐点云。GICP 发布 `map → odom → livox_frame` 分支中的全局校正，导航适配器发布独立 `map → odom_nav → base_footprint` 分支。首次成功定位后，偶发或连续的配准拒绝会在 /gicp/status 记录 HOLDING_LAST_FIX、点云时间 scan_stamp_ns、上次定位年龄 last_fix_age_s 和拒绝原因。默认沿用最后一次地图校正，机器人位姿由实时里程计更新。速度门控继续检查有效的定位、实时 TF、扫描、里程计和命令。长时间只用里程计会积累漂移。如需限制沿用时间，将 GICP 配置中的 transform_timeout 设为正数（秒）。GICP 节点更换时重启整套导航，以清除旧定位来源。

该入口的输出边界为 `/cmd_vel`。实车部署还需核对安装外参、录包或现场定位重复性、当前地图规划，以及底盘速度接口。GICP 从启动区初猜附近匹配，场地真实摆放容差需实测。

## 隔离回归

```bash
source run/environment.sh
ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  /usr/bin/python3 validation/test_navigation_runtime.py --gicp
ROS_DOMAIN_ID=195 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  /usr/bin/python3 validation/test_navigation_runtime.py
```

使用合成地图、合成点云和真实 Nav2 / GICP 节点，覆盖规划、速度输出、定位拒绝与断流归零。

迁移包附带的较大点云测试在本机使用 CycloneDDS 的进程级分片配置：

```bash
source run/environment.sh
ROS_DOMAIN_ID=196 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  CYCLONEDDS_URI="file://$PWD/validation/cyclonedds_gicp_test.xml" \
  /usr/bin/python3 lidar_ws/src/go2_real_localization/test/synthetic_check.py
```
