# 独立 GICP 重定位

本入口只启动 Livox、FAST-LIO、CPU FastGICP 和 RViz，不启动串口、踩点或路线控制。
原 start.sh 保持原用途，本阶段不要用它启动重定位实验。

## 编译与启动

```bash
cd ~/letsgo
bash run/build_ros2.sh
bash run/gicp.sh
```

WSL 当前已验证的通信环境（2026-10-29）：启动前在每个相关终端执行
`export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0`。此项目使用 `source run/environment.sh`，
不要 source 旧 go2_ws/install/setup.bash。不是所有 ROS 部署都必须使用域0；
这里只记录本机实测可用组合，不修改其他机器的全局 ROS 配置。

```bash
cd ~/projects/letsgo
source run/environment.sh
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
bash run/gicp.sh
```

## 2026-10-29 验证记录

- 本项目5个包编译通过；10项C++核心用例通过。
- 实际PCD加载成功，降采样后42440点。
- 在域0、LOCALHOST_ONLY=0下，synthetic_check.py 和 runtime_check.py 均退出码0。
  验证了合成扫描配准、map→odom组合、无关点云拒绝、恢复、输入超时、
  零时间戳、ROS时钟暂停及前后跳变。这些是合成数据验证，不是实机精度测量。
- 之前其他域/仅本机组合出现跨进程消息超时；根因没有进一步定论。
- 真实雷达→FAST-LIO→GICP全链路尚待现场验证；必须检查
  /cloud_registered_body（不是/world系的/cloud_registered）和/Odometry，
  给定初始位姿后观察/gicp/status的ACCEPTED及/gicp/valid。

已有 FAST-LIO 时只启动定位模块，避免重复启动驱动：

```bash
bash run/gicp.sh --localizer-only
```

无图形显示可追加 `show_rviz:=false`。默认地图在项目 maps/ 中，换地图用
`GICP_MAP_PATH=/绝对路径/地图.pcd bash run/gicp.sh`。运行不依赖 go2_ws。

## 定位步骤

1. 雷达必须位于这份 PCD 对应的真实场景，先等待 FAST-LIO 初始化。
2. 在 RViz 的地图上用 2D Pose Estimate 指定机体大致位置和朝向。
3. 查看 /gicp/status 和 /gicp/valid。valid=true 表示当前结果通过质量与时效检查，不是实测绝对精度保证。
4. 输出 /gicp/pose、/gicp/aligned_cloud，以及 map→odom TF；没有修改原 /Odometry。

这是有初值定位，不是无初值全局搜索。高度或倾斜明显时，2D 初值可能不足，需通过
/initialpose 提供合适的三维位姿。不要把 PCD 原点自动当作每次开机位置。

## 参数和负载

参数集中在 lidar_ws/src/go2_real_localization/config/gicp.yaml。
沿用原实机参数：体素0.10m、配准周期1s、局部半径25m、4线程、40次迭代。
独立 worker 执行匹配。配准质量、超时、修正幅度检查沿用旧实现。
完整 GICP 入口关闭额外累计 /Laser_map 和 PCD 保存，保留 FAST-LIO 内部地图、
/cloud_registered_body 及 /Odometry。仅定位入口不修改已运行 FAST-LIO 的参数。
地图显示使用 /gicp/map。GICP 会增加计算量，需要 Jetson 实机评估耗时。

## 来源

迁移自 /home/ice/go2_ws/src/unitree-go2-ros2/go2_real_localization 和 fast_gicp。
保留原包名、源码与许可证；运行依赖和启动入口改为本项目。
PCD 为 real_room_20260926_110555_607459.pcd，约147万点。
当前未接入踩点、导航或电控，后续可另行设计全部采用 PCD/map 坐标的路线。
