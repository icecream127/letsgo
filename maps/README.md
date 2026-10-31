# 地图工作流

地图统一保存在项目根目录 `maps/<地图名>/`。地图名允许字母、数字、下划线和短横线。

```text
maps/competition/
  cloud.pcd          # FAST-LIO 累积点云，采集时 odom 坐标
  map.pgm            # 二维占据栅格图
  map.yaml           # Nav2 map_server 入口，image: map.pgm
  projection.json    # 投影参数、坐标说明、文件校验值
```

## 先构建

在 Ubuntu 22.04 / ROS 2 Humble 中，于项目根目录执行：

```bash
rosdep install --from-paths lidar_ws/src --ignore-src -r -y --rosdistro humble
bash run/build_ros2.sh
```

新增依赖包括 nav2_map_server、nav2_lifecycle_manager、python3-numpy、python3-yaml。

地图入口脚本固定使用 `/usr/bin/python3`，避免 Conda `(base)` 中的 Python 缺少 YAML 或无法加载 ROS Python 模块。若系统 Python 仍缺依赖，执行 `sudo apt install python3-yaml python3-numpy`；不需要在 Conda 中安装 ROS 依赖。

## 1. 采集并保存 PCD

```bash
bash run/mapping.sh competition
# 无图形界面可加 show_rviz:=false
```

只启动原有雷达、FAST-LIO 和可选 RViz；不启动串口、旧路线或电控。脚本根据自身位置计算绝对地图路径，自动创建目录，并覆盖 FAST-LIO 的保存参数；不受终端当前目录影响。普通 lio_driver.sh 的默认保存仍关闭。

人工缓慢移动机器人采集。保持上述节点运行，另开终端在项目根目录执行：

```bash
bash run/save_map.sh competition
```

保存客户端确认当前 FAST-LIO 启用了对应目录，再调用 /map_save，并校验生成的 PCD。空地图和写入错误返回失败；PCD 通过临时文件写入后替换。已有文件默认拒绝覆盖，如需更新使用 `save_map.sh competition --force`，更新后必须重新生成二维地图。

这是当前 FAST-LIO 每秒累积当前帧的地图，不是回环优化后的全帧融合地图。必须在关闭节点之前主动保存。

## 2. 生成二维地图

在点云查看工具中确定地面在这份 PCD 中的 Z 坐标。**不能把雷达离地高度直接当作 ground-z；启动姿态、IMU 原点和外参都会影响它。** 以下 -0.35 仅是示例，必须替换：

```bash
bash run/map_to_2d.sh competition --ground-z -0.35
```

默认：分辨率 0.05 m；地面厚度 ±0.05 m；高于地面 0.10～2.0 m 的点标为障碍；边界留 0.5 m。可选 `--resolution`、`--ground-tolerance`、`--min-height`、`--max-height`、`--padding`。重做时显式加 `--force`。

- 支持本项目保存的 binary PCD 和 ASCII PCD；不支持 binary_compressed。
- 地面观测所在格子标为空闲；障碍优先；未观测格子保持未知。
- 不根据点云包围盒推断整片空地，不推断射线穿过的空闲区域。点云稀疏时可能出现未知孔洞，应改善采集或调整分辨率，不能据此宣称路线可达。
- 只适合相对平坦地面的初版投影。坡道、台阶、悬空障碍及可跨越区域需要专门处理。
- X/Y 直接沿用保存点云坐标，不引入旧 /pos_trans 的 Y 反射；PGM 上下翻转与 YAML 左下角 origin 配套。
- 原始 PCD 不修改。生成记录保存源文件与输出校验值，避免更新点云后误载旧二维地图。

## 3. Nav2 加载

```bash
bash run/nav2_map.sh competition
```

先验证文件一致性，然后启动官方 Nav2 map_server 及 lifecycle manager，自动激活并发布 `/map`（frame_id: map）。RViz 的 Fixed Frame 设为 `map`，添加 Map 显示；QoS 选择 Reliable / Transient Local。

```bash
ros2 lifecycle get /map_server
ros2 topic echo /map --once --field info --qos-durability transient_local --qos-reliability reliable
```

此入口仅接入地图加载，不启动 AMCL、规划器、控制器，也不会输出或发送运动命令。后续完整 Nav2 可直接使用同一个 `maps/competition/map.yaml`。

`map` 的数值轴对应采图时 FAST-LIO 的 odom。重新启动 FAST-LIO 后的实时 odom 未必与保存地图对齐；后续必须由定位系统或经过确认的初始对齐提供 map → odom。不要因为地图加载成功就默认两者同原点，也不要与另一个 map_server 重复启动。

## 离线验证

```bash
python3 -m pytest validation/test_maps.py -q
```

已构建项目后，可在无其他节点的独立 ROS domain 验证实际 PCL 保存函数和 Nav2 map_server，以及 FAST-LIO 空地图拒绝行为：

```bash
source run/environment.sh
ROS_DOMAIN_ID=189 ROS_LOCALHOST_ONLY=1 LETSGO_MAP_RUNTIME_TEST=1 \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  python3 -m pytest validation/test_map_runtime.py -q
```

运行验证使用临时合成点云，不连接雷达或电控；需 C++ 编译器和 PCL 1.12 开发库。

本机 WSL 中默认 Fast DDS 曾出现服务已发现但请求持续超时，地图服务端日志仍显示激活成功；切换 Cyclone DDS 后上述测试通过。验证命令需要已安装 `ros-humble-rmw-cyclonedds-cpp`。未修改运行脚本的默认 RMW；如现场也遇到此问题，应在建图和保存等所有相关终端统一设置 RMW_IMPLEMENTATION，再重新启动节点。

仓库不附带伪装成现场采集的地图。首次运行需采集真实 cloud.pcd，再生成并检查二维地图。
