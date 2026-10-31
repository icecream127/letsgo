# Nav2 输出速度阶段

此入口启动 MID360、FAST-LIO、点云转 LaserScan、AMCL 和官方 Nav2 节点，输出 geometry_msgs/Twist 类型的 `/cmd_vel`。不启动串口、电控或原项目的区域任务控制。

## 运行步骤

完成新地图的保存及二维投影后，在 WSL 中运行（competition 换成新地图名称）：

```bash
cd /home/zxy/letsgo
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
bash run/navigation.sh competition
```

不要同时运行 mapping.sh、lio_driver.sh、nav2_map.sh 或 start.sh，以免重复传感器、TF、地图服务或旧控制节点。若已单独启动雷达和 FAST-LIO，改用 `bash run/navigation.sh competition sensors:=false`。所有参与节点、RViz 和检查终端应使用相同 ROS_DOMAIN_ID 和 RMW_IMPLEMENTATION；更改 RMW 后先退出旧节点再重启。

1. 保持雷达静止，等待 FAST-LIO 完成初始化。
2. 在 RViz 设 Fixed Frame 为 `map`，使用 `2D Pose Estimate`，在地图上点击当前位置并拖动朝向。AMCL 用它作为初始估计，再用实时 `/scan` 匹配地图。当前没有自动全局重定位。
3. 检查 `/scan` 与墙面是否重合。无初始位置时，Nav2 等待 map TF 属正常现象。位置不对就重新指定，不能仅凭节点启动成功认定定位正确。
4. 定位正确、Nav2 激活后，用 RViz 的 `Nav2 Goal` 指定近处空闲目标。
5. 另开终端查看速度：

```bash
cd /home/zxy/letsgo
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source run/environment.sh
ros2 topic echo /cmd_vel
```

诊断命令（同一环境）：

```bash
ros2 topic echo /navigation/status
ros2 topic hz /scan
ros2 topic hz /odom
ros2 lifecycle get /amcl
ros2 lifecycle get /bt_navigator
ros2 topic info /cmd_vel -v
```

`ready` 表示速度门控条件满足；其他状态说明等待 TF、定位或新鲜数据。没有目标时命令超时输出零是正常的。`/cmd_vel` 应只有 `nav_cmd_gate` 一个发布者。取消目标用 RViz 的 Cancel；结束程序用 Ctrl+C。进程退出后不再持续发送零速度；将来接电控仍需电控端通信超时保护。

## 地图与坐标

- 读取 `maps/<名称>/map.yaml`，二维图和 PCD 的哈希验证复用已有脚本。Nav2/AMCL 使用二维图，不直接加载 PCD 定位。
- 自动倾斜校正代码及配置已撤销；不修改已有地图，也不采用之前预览地图的校正矩阵。新地图应保持适用于二维导航的地面坐标。
- TF：AMCL 发布 `map -> odom_nav`；适配器连接 `odom_nav -> odom -> livox_frame` 并发布平面 `odom_nav -> base_footprint -> nav_scan`。FAST-LIO 保留原 TF。
- `/Odometry` 经平面化后成为 `/odom`，速度由短时间窗口的位置和朝向差分估计；不是轮速反馈。
- 初始雷达坐标在每次启动时重置，因此每次重新启动需重新初始化 AMCL。

## 暂定参数

配置文件：`lidar_ws/src/ros2_to_stm32/config/nav2.yaml`。

- `livox_to_base_xyz` 和 `livox_to_base_yaw` 默认零：暂以传感器原点作为机器人平面中心，必须按实际安装测量后再用于真车。
- 两个 costmap 的 `robot_radius` 暂为 0.30 m，需按车体尺寸确认；不是已测量值。
- 当前只规划前进/转向，不输出横向速度；门控限幅 x 为 -0.15～0.25 m/s、角速度 ±0.6 rad/s。
- 实时扫描默认选择雷达原点下方 0.3 m 到上方 1.0 m 的点，不是地图 `ground-z`。可用 `scan_min_height:=-0.3 scan_max_height:=1.0` 调整；依据安装高度排除地板并保留墙/障碍，检查扫描与二维图的一致性。该默认值不保证能看到低矮障碍。
- 规划器禁止穿越地图未知区域。如果可行驶地面大量为灰色，应先改善地图，不应直接把全部未知区域改为空闲。
- 输出阶段机器人不自动移动，因此给目标后可能因“没有进展”而取消/进入恢复；这是无电控反馈时的正常限制。
- `/cmd_vel_nav` 为 Nav2 原始速度，`/cmd_vel` 为检查新鲜度后的最终速度；没有有效 AMCL 位姿/TF、odom 或 scan 超过 0.5 秒不更新、原始命令超过 0.5 秒不更新时输出零。此门控不是定位精度或防碰撞保证。

## 验证

本机 ROS2 Humble 下构建通过。集成测试用合成房间地图、模拟 FAST-LIO 里程计与 PointCloud2，实际启动点云转换、AMCL、Nav2 规划器/控制器，验证非零 `/cmd_vel`、唯一发布者、取消目标归零、传感器断流时阻止持续原始速度。没有启动硬件驱动或串口，没有验证新地图或真实 MID360 导航效果。

```bash
cd /home/zxy/letsgo
source run/environment.sh
export ROS_DOMAIN_ID=195 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=1
/usr/bin/python3 validation/test_navigation_runtime.py
```

测试日志：`/tmp/letsgo_navigation_test.log`。测试结束会退出其启动的节点。此 ROS_DOMAIN_ID 只用于隔离测试，正常运行不要混用。该 WSL 环境中 CycloneDDS 测试出现点云无法送达，Fast DDS 下完整流程通过，因此这里明确使用已验证的 Fast DDS。
