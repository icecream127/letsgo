# GICP 接入走点（2026-11-19）

## 链路与边界

FAST-LIO 发布 odom→livox_frame；GICP 发布 map→odom。
新增 gicp_pos_trans 以 20 Hz 查询 map→livox_frame，并沿用原 trans.py 的 A/B
赛道转换，发布 /pos_trans。info_trans（或 mini）原有 yaw 取负和 0.21 m
身体中心补偿仍只执行一次。没有更改电控协议、GICP 算法及原有路线。

不是直接订阅约 1 Hz 的 /gicp/pose：使用 TF 链得到连续位姿，同时检查
/gicp/valid、/Odometry 和 TF 时效。失效时停发 /pos_trans，现有串口节点
pose_timeout=1.0 s 后请求站立；这不是瞬时硬件急停。一次性动作能否中断由
电控决定，必须保留实体急停和现场监护。

## 使用（每个终端先设相同 ROS 环境）

```bash
cd ~/projects/letsgo
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
bash run/gicp.sh
```

等 IMU 初始化，RViz 2D Pose Estimate 给初猜，确认持续 ACCEPTED、valid=true。
另一个终端：

```bash
cd ~/projects/letsgo
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
bash run/gicp_pos_trans.sh
```

不要运行原 pos_trans.sh、start.sh 或额外 lio_driver.sh：避免两套 /pos_trans、
重复驱动。本次不自动启动串口和路线。需要使用旧 UI 重新采 A/B，建立地图到
赛道的关系；旧路线点仍是赛道坐标，不是 PCD/map 坐标。停止或重启本适配器
后需要重新采 A/B。重新设置 GICP 初始位姿前先解除电控使能，检查对齐并重采。

先在不使能电机时启动原 UI/串口并核对 /pos_trans、坐标轴方向和身体中心补偿。
确认后只启动一个 info_trans 或 tasktodo_mini 控制器。原来的区域位置必须对应
当前 A/B 赛道，不能把别的赛场旧路线直接用于这张房间 PCD。

## 验证与限制

执行 `source run/environment.sh` 后运行
`python3 validation/gicp/test_course_bridge.py`。
测试地图位姿到赛道位置/yaw、未踩点、失效、超时等；不代表真实马已验收。
实机需验证移出/遮挡雷达后定位失效触发站立，以及受监护低速两点走行。
没有修复任何真实 FAST-LIO 漂移，GICP 依赖里程计正常，不能以此掩盖时间戳问题。

## 文件与回退

- 新增 ros2_to_stm32/gicp_trans.py、run/gicp_pos_trans.sh、验证脚本。
- setup.py 注册 gicp_pos_trans；package.xml 声明 tf2_ros_py。
- 旧 pos_trans、控制器、串口源码未修改。回退时停掉 GICP 位置适配器，换回
  run/pos_trans.sh 并重新采 A/B，不能同时运行两者。
- 更新前源码备份：/home/ice/letsgo_before_gicp_update_20261119.tar.gz。
- GICP 源码、适配器和默认 PCD 随仓库提交；其他设备拉取后需重新编译，
  不能只拉取源码就继续运行旧 install 中的程序。
