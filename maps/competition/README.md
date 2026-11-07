# 完整制图工作流：点云采集 → 二维底图 → GIMP 修订 → 导入 → 验证

## 1. 当前正式地图与目录

2026-11-07 整理。当前 map.pgm 是用户在 GIMP 修改后，从 Windows 桌面 navmap/map.pgm 原样导入的版本。配套 cloud.pcd 已归并到本目录，它是原 real_room_20260926_110555 的实际源点云，不是此前 competition 目录中的另一份点云。

```text
maps/
└── competition/
    ├── README.md             # 本文
    ├── cloud.pcd             # 与地图匹配的原始点云
    ├── map.pgm               # 当前正式人工修订版
    ├── map.yaml              # Nav2 加载入口
    ├── projection.json       # 源点云、图像、YAML 校验与修改记录
    └── traversability.json   # 本场地下方坡板及 C 入口的专用通行区域
```

当前尺寸 260×260；分辨率 0.05 m/格；origin=[-1.9,-7.25,0]。坡度上限 15°、单级高度 0.10 m。机器人半径 0.30 m，Nav2 膨胀半径 0.32 m。

源点云 SHA256：`6edd690d2569b91bbd6be1b2a7b4534c96ea5cb5085fa0535fd3ec0c6c7ed47e`。
人工图 SHA256：`cd729af9731dc42458a3f78c5f70fa5edc6f137e59a8a6e85cf8ae361afa23dd`。

验证报告和预览图位于项目 `validation/results/competition/`，不放在 maps 中。旧预览地图、旧点云、重复图像和历史备份已移出 maps，归档到 `.local/maps_cleanup_archive/20261107/`。批量递归删除被自动审批策略阻止，因此采用可恢复的移出归档。

**使用当前地图直接执行第 7 节，不要重新投影。投影生成的是自动底图，不会自动保留 GIMP 修改。**

## 2. 环境准备

以下命令全部在 Ubuntu/WSL 终端执行，默认项目根目录：

```bash
cd /home/zxy/letsgo
source /opt/ros/humble/setup.bash
```

首次部署或依赖缺失时：

```bash
sudo apt install python3-numpy python3-yaml python3-pil python3-matplotlib python3-pytest
rosdep install --from-paths lidar_ws/src --ignore-src -r -y --rosdistro humble
bash run/build_ros2.sh
```

使用系统 `/usr/bin/python3`，避免 Conda Python 与 ROS 环境不一致。原生机器人上项目路径可能为 /home/jetson/letsgo，请替换 cd 路径。

## 3. 新场地采集并保存点云

当前 competition 已有正式数据。新采集使用新名字，例如 session_20261107；mapping.sh 会拒绝在已有 cloud.pcd 的地图目录直接开始采集。

终端一：

```bash
cd /home/zxy/letsgo
bash run/mapping.sh session_20261107
# 无图形界面可在末尾添加 show_rviz:=false
```

缓慢移动机器人，重点观察入口、墙边、坡道两侧和遮挡区域。快速转动、人员经过和缺少侧面观测都可能留下错误回波。仅有单次扫描不能保证坡板轮廓完整。

保持采集节点运行，在终端二保存：

```bash
cd /home/zxy/letsgo
bash run/save_map.sh session_20261107
```

保存成功后再关闭采集。输出 maps/session_20261107/cloud.pcd。已有点云默认不覆盖；只有确实准备更新时才使用 --force。当前保存是 FAST-LIO 累积点云，不是回环优化后的完整重建。

## 4. 从点云生成二维底图

新点云应先确定起始地面 XYZ。ground-z 是保存点云中的地面 Z，不是直接填写雷达安装高度。新场地不能照搬本场地的裁剪范围、种子位置和通行区域。

新场地示例（必须将地面和种子位置替换为实测值）：

```bash
bash run/map_to_2d.sh session_20261107 \
  --ground-z -0.75 --seed-x 1 --seed-y 0 \
  --resolution 0.05 --max-step 0.10 --max-slope 15
```

只在需要重做当前场地底图时，先备份本目录到 maps 之外：

```bash
backup_dir=".local/map_backups/competition_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$backup_dir"
cp -a maps/competition/. "$backup_dir/"
```

再执行当前点云对应的投影参数：

```bash
bash run/map_to_2d.sh competition \
  --ground-z -0.75 --seed-x 1 --seed-y 0 \
  --resolution 0.05 --max-step 0.10 --max-slope 15 \
  --bounds -1.9 -7.25 11.1 5.75 --force
```

该命令现在直接读取 competition/cloud.pcd，不需要 --source-map。投影脚本覆盖时也会生成目录内 backups；这些是以后运行新产生的备份，不属于本次清理的现有内容。

算法按输出格的密集高度带估计表面，检查坡度和相邻高度差，保留墙体及落差边缘。未知区域不默认全白。--terrain-resolution 和 --ground-quantile 仅作旧命令兼容，现地形网格跟随 --resolution。

traversability.json 只适用于当前源点云：照片显示下方为有真实缝隙的分段坡板，用户确认底盘已能从 C 进入并通过，最大缝宽未测量。因此区域内可按底盘通行能力表达缝隙，并保留两侧边界与 C 开口；白色不代表缝隙内存在实体地面。该能力不能推广到其他沟槽，普通二维图也不能表达强制单向通行。

换点云时其校验值会不匹配：不要只修改哈希绕过检查。应重新核实区域，或在 maps 之外保存旧规则，在新地图目录先不放 traversability.json。

## 5. 使用 GIMP 修图

Windows 文件资源管理器打开：

```text
\\wsl.localhost\Ubuntu-22.04\home\zxy\letsgo\maps\competition
```

打开原始 map.pgm，不打开带坐标轴、文字、路线的截图。另存 XCF 工作文件到桌面等工作目录，避免混入正式地图目录。

1. 图像 → 模式 → 灰度；保持 8 位灰度、260×260。新场地则保持它自己的尺寸。
2. 可以提高查看缩放比例；不要执行图像缩放、旋转、裁剪，也不要移动整层。
3. 工具 → 绘画工具 → 铅笔（N）。使用实心笔刷 2. Hardness 100；模式正常、不透明度 100%；关闭“启用动态”。细修先用 1～3 像素。
4. 点击工具箱两个叠放方块中的左上前景色方块，在 HTML 标记输入颜色。

| 内容 | HTML 颜色 | 灰度 |
|---|---|---:|
| 可通行 | FEFEFE | 254 |
| 障碍或不能驶出的边界 | 000000 | 0 |
| 未知 | CDCDCD | 205 |

按 X 可交换前景/背景色；按 Ctrl+Z 撤销。画直线：点击起点，按住 Shift 点击终点。不要用橡皮擦制造透明区域。大区域可用关闭羽化和抗锯齿的选区填充。

修入口时既要清除入口内的误障碍，也要保留真实两侧边界。1 格=5 cm，12 格=60 cm，等于车体直径但没有余量；必须按真实净宽画图，不能为了规划通过而虚构加宽。

如果铅笔无反应：先确认左侧标题为“铅笔”，笔刷不是“剪贴板图像”，动态关闭；右侧通道面板的“灰度”行必须选中高亮，眼睛仅表示可见；再切到图层，选中未锁定的地图图层。测试时在黑色区域点一下白色，成功后撤销测试点。

导出：文件 → 导出为 → map_edited.pgm；PGM 对话框选择“原始”（二进制）。建议保存到 Windows 桌面 navmap，保留原正式图。导出应无透明、无文字、无路线，只有灰度 0/205/254。GIMP 的 Ctrl+S 保存的是工作文件，PGM 必须导出。

官方参考：https://docs.gimp.org/3.0/en/gimp-tool-pencil.html

## 6. 正式导入 GIMP 修订图

新增导入脚本会检查原地图、源点云、图像尺寸、8 位灰度和三色像素，保持 YAML 的坐标不变，备份旧版本，并更新 projection.json。不需要手工改哈希。

先检查（不写入）：

```bash
/usr/bin/python3 run/import_map_image.py competition \
  /mnt/c/Users/22512/Desktop/navmap/map_edited.pgm --check-only
```

再导入：

```bash
/usr/bin/python3 run/import_map_image.py competition \
  /mnt/c/Users/22512/Desktop/navmap/map_edited.pgm
```

若导出文件仍叫 map.pgm，替换命令中的文件名即可。输入必须是另存文件，不能先覆盖正式 map.pgm 再导入。

旧图保存在 `.local/map_backups/<地图名>/<UTC时间>/`，不混入 maps。导入后旧规划报告会标记为 STALE，需要重新验证。再次投影不会合并这次人工修图；对外部署请携带本目录完整配套文件。

## 7. 校验与仅加载地图

```bash
/usr/bin/python3 run/maps.py validate competition
bash run/nav2_map.sh competition
```

脚本校验 cloud.pcd、map.pgm、map.yaml，以及存在的区域配置校验值。仅启动 map_server 和 lifecycle manager，不启动控制器或电控。

RViz：Fixed Frame=map；添加 Map，Topic=/map，Reliability=Reliable，Durability=Transient Local。

```bash
ros2 lifecycle get /map_server
ros2 topic echo /map --once --field info \
  --qos-durability transient_local --qos-reliability reliable
```

地图加载不代表实时定位已对齐。正式导航需正确的 map→odom 定位关系，不能把重新启动后的 odom 直接当原采图坐标。

## 8. 规划验收

离线算法回归：

```bash
/usr/bin/python3 -m pytest validation/test_maps.py -q
```

本场地下方过道的圆形车体检查（半径 0.30 m，计入栅格角点，未知格不可通行）：

```bash
/usr/bin/python3 validation/verify_competition_terrain_route.py
```

隔离 ROS domain 验证真实 Nav2 全局规划服务，沿用项目的 NavfnPlanner、半径 0.30 m、膨胀 0.32 m：

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=189 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  /usr/bin/python3 validation/verify_nav2_terrain_planner.py --c 4.925 -4.075
```

需要可用且空闲的 domain 189，以及已安装 Cyclone DDS。若缺少该 RMW，安装 ros-humble-rmw-cyclonedds-cpp。脚本只启动地图、静态测试 TF、全局规划器和生命周期管理，不驱动实车。

当前人工图：旧 C=(5.00,-4.20) 落入膨胀区。测试使用 C=(4.925,-4.075)，场内→C、C→A 均通过，目标误差为 0。这只是测试目标，没有更改实车任务点。测试坐标和过道范围均为本场地专用，新场地必须重新选点。

报告在 validation/results/competition/。报告中的 map_sha256 必须与当前图一致；过去某两段路线通过，不能替代所有入口及任务路线的检查。实车还需要低速验证定位、控制和实际净空。

## 9. 常见问题与回退

- 校验失败：确认没有直接覆盖正式 PGM，使用第 6 节导入。点云变更则需重建底图。
- 图像尺寸不匹配：回 GIMP 撤销图像缩放/裁剪，不能靠改 YAML 分辨率补救。
- 出现其他灰度：关闭羽化、抗锯齿、动态和半透明绘制，恢复为指定三色。
- 入口看起来白但规划失败：检查膨胀后的全局代价地图和目标点位置；真实入口过窄不能通过擦图解决。
- 新点云被旧区域规则拒绝：重新核实区域，不能只改规则中的源哈希。
- 需要回退：从同一个备份批次恢复 map.pgm、map.yaml、projection.json、相应 traversability.json；源 cloud.pcd 也必须与记录一致，然后再次 validate。
- 完成新场地后只保留正式目录：先核验正确 PCD 已在该目录内，再更新来源记录、清理试验目录；不要删掉仍被 projection.json 引用的外部点云。
