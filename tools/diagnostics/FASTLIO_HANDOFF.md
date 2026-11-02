# 最新交接：2026-11-01 21:40 后单实例复测

以下为本轮更新；文末旧交接中的 PID、运行状态和结论均为历史记录，不能直接沿用。
证据目录：`tools/diagnostics/results/current/`。当前 git HEAD `b034f517`，分支 zxy。

最后核对：2026-11-01 约21:49 CST。

## 当前结论：A、B 均有独立证据，最终 Nav2 验收未通过

- **A，启动阶段真实等待已证实**：原驱动两个发布线程固定 sleep 3 s，首个 IMU 发布汇总最大年龄 3015.841 ms（`startup_sleep_baseline/driver_final.log`）。移除此等待后，仍有设备配置阶段等待，不能称启动延迟完全修复。
- 最终单实例启动：首包 steady=2364259104490 ns，Sampling-ready=2366146547029 ns，相差 1887.443 ms；首个 IMU 接收=2364260213582 ns、publish 开始=2366153946246 ns，**真实等待 1893.733 ms**。首个 IMU 年龄约 1892.7 ms。队列在设备进入 Sampling 后开始分发；未清队列或丢帧。设备在驱动重启间持续输出，配置完成前已有数据进入队列。
- **B，输入时间基准异常已实测**：最终进程的 IMU 持续约超前主机 113 ms；Odometry 也超前约 68 ms。首个锚点以后约 5 s 的 `device_delta - steady_delta` 达 116.731 ms，此后维持约 116 ms。主机 wall/steady 无对应跳变，FAST-LIO 队列没有持续老化。因此不能把负年龄和及时处理当作时间戳正确。
- **尚未解释**：这 116 ms 是首次设备包陈旧/SDK 启动排队，还是设备时钟在启动期间前跳。现有每 5 s 样本不足以区分；没有把它直接认定为此前 4120 s 的根因。
- **运行中正向 1～3 s Odometry 老化、4120 s 共同偏移：本轮尚未复现，根因未解决。** 不宣称时间映射已修复。
- 另一次 IMU publish 耗时 214.418 ms，队列最大等待 216.438 ms，随后恢复（日志 stamp_ns=1793540583919022020）。steady 包围 publish 的耗时含线程失调度，不能单凭它确认 DDS 阻塞；QoS/线程审计见下。
- Nav2 的明确阻碍：用户确认真实位置就是旧地图黑区，起点全局 cost=254；真实起点规划返回空路径。不得虚构/挪动初始位姿或清障来通过验收。

## 范围、进程及构建产物

本轮开始宿主未发现驱动、FAST-LIO、Nav2、RViz。已重新启动，未沿用旧 PID。
最终仅一份驱动 PID **140562**（21:40:15 启动）和一份 FAST-LIO PID **140635**（21:40:17 启动）。
Nav2、nav_adapter、nav_cmd_gate、pointcloud_to_laserscan 和 RViz 保持运行；导航目标已取消，最终速度为零。
`navigation_evidence_observer` 继续只读记录 `/initialpose` 等，初设运行 3600 秒后退出。
外部 300 s 探针已完成；驱动/FAST-LIO 的稀疏诊断继续输出。再次接续必须重新查 PID。

`provenance_final.json` 核对了 `/proc/PID/exe`、映射模块 inode、磁盘 inode、SHA256 和源码散列。
驱动主要逻辑实际在动态库，**仅核对启动器 hash 不够**：

- FAST-LIO 可执行文件 hash：`6051b9bdb38163ea4d7ae5acea943ff6f463240bea97e3df8a21e212400c6854`。
- 运行驱动 `liblivox_ros_driver2.so` hash：`72d95c267d6a4cf64bfdab0357a0a899deff8398627a35d5dbf827c94d3597e4`，映射与磁盘 inode 都是 1006222。
- 实际 SDK：`.local/lib/liblivox_lidar_sdk_shared.so`，API 报版本 1.3.1；hash `6decd7e5b0bd0d27bf1b31dacba9d7865dcc58103fdb117f43dd75bb4a806819`，与 `.build_sdk/sdk_core/` 产物一致。源码位于 `third_party/Livox-SDK2`；SDK 未改动/未重建。
- 最终参数 dump：`fastlio_final_params.yaml`。0.3 m 体素、IMU depth 200、关闭 map/path/PCD 保存；保留 world/body 点云；use_sim_time=false，common time_sync_en=false、IMU offset=0。
- master...zxy 差异再次核对：驱动无分支差异，FAST-LIO 分支差异集中地图保存；本轮本地诊断、sleep 删除另行列出。未进行 master A/B，不把体素/版本差异混测。
- 重启核验曾发现旧 FAST-LIO PID 123617 对 INT/TERM 未完整退出，清理后才开始最终单实例窗口；`ready_diag_before_queue_copy` 属中间诊断，不作最终单实例验证。未为这一旁支改 FAST-LIO 信号处理。

## 时间来源与测量约定

1. `pub_handler.cpp:GetEthPacketTimestamp`：以 memcpy 读取设备包 64-bit ns；本轮 timestamp_type=0 (NoSync)。type 1/2 是 PTP/GPS，源码直接返回设备 epoch 时间，本轮未观测到。
2. NoSync 原转换：`mapped_ns = anchor_epoch_ns + raw_device_ns - anchor_device_ns`。LiDAR 与 IMU 按 handle 共用同一锚点；锚点在驱动进程内首次收到设备包时建立。新驱动进程重建锚点，不能说进程重启后复用了旧锚点。
3. 仍有源码风险：同一驱动进程内设备重启/重连，不见清除或重新鉴定旧 anchor 的逻辑；系统时间跳变也不会同步更新锚点。尚未在本轮触发这些事件，不能据此认定历史根因。
4. 点云 `base_time/timebase/header.stamp` 均为首点时间；逐点 `offset_time = point_ns - pkg.base_time`，uint32 ns。CustomMsg 的序列化头和末点 offset 用 CDR 定位读取，没有 Python 解码点数组。
5. FAST-LIO `livox_pcl_cbk` 取 header 为 lidar_beg_time；预处理将 offset_time / 1e6 存 curvature(ms)；`sync_packages` 用末点 curvature/1000 得 lidar_end_time（异常短帧有均值回退）。扫描时长约 100 ms，不能当计算耗时。
6. `publish_odometry` 直接使用 `get_ros_time(lidar_end_time)`，不是 now。点云 body/world 也用帧尾时间。
7. 年龄是 ROS/system now 减 measurement stamp；跨进程阶段关联按同一 base_ns（浮点舍入容许 <512 ns），使用 std::chrono::steady_clock 的共同基准。已确认两进程相同 time namespace，C++ steady 与 CLOCK_MONOTONIC 相差仅调用间隔量级，探针 time.monotonic 同基准。原 omp 内部耗时只在同线程内作差，未与 steady 的绝对值相减。
8. 当前新增日志仍是约每 5 s 稀疏阶段样本，不代表所有帧的 P95；Odometry+TF publish 合并计时、clouds 合并计时，**尚未将 FAST-LIO 每个发布调用完全拆开**。若需要定位新的持续慢发布，需补线程 CPU/调度与逐 publisher 统计，不能称全部阶段已穷尽。

## 最终 300 s 窗口

从正常启动开始覆盖，而非只截取稳定尾部。完整 300 s：点云 2984、IMU 59379、Odometry 2932 条；
点云启动期最大年龄 1.764 s、15 条超过 0.5 s；驱动内部 IMU 首条 1.893 s 等待由 native 诊断捕获，外部 DDS 订阅发现较晚，不能用外部首条替代驱动首条。
完整 Odometry 最大正年龄 134.9 ms，无 >0.5 s 样本。时钟偏差存在，所以这些数值不能当成真实端到端延迟。
完整结果 `final_30s.jsonl`、帧关联 `final_frames.jsonl`、冻结汇总 `summary_300s.json`。

以下明确排除首个 30 s，展示后续约 270 s；完整启动数据未删：

| 话题 | 条数 | 平均年龄 ms | 最大正向年龄 ms | >0.5s |
|---|---:|---:|---:|---:|
| /livox/lidar | 2699 | -1.91 | 137.86 | 0 |
| /livox/imu | 53944 | -112.65 | 106.64 | 0 |
| /Odometry | 2700 | -67.69 | 134.88 | 0 |

三路均无时间戳回退。负年龄表明 B 类偏差，不是“延迟为负所以更快”。
首轮旧参数/旧产物180 s则是 Odometry平均38.9 ms、P95 54.2 ms、最大241.3 ms（`startup_180s.jsonl`），不能替代最终版本测试。

同帧稀疏关联约 61 组，真实阶段耗时（ms）：

| 阶段 | 平均 | P95（稀疏样本） | 最大 |
|---|---:|---:|---:|
| 首包回调到组帧（含采集） | 100.60 | 108.52 | 114.13 |
| 组帧到驱动 publish 开始 | 5.10 | 16.07 | 28.75 |
| publish 开始到 FAST-LIO 回调入口 | 9.20 | 20.39 | 45.95 |
| 回调入口到同步/处理开始 | 3.60 | 9.45 | 20.82 |
| 处理整帧 | 30.62 | 46.90 | 49.20 |

处理采样：预处理平均 0.57 ms；去畸变/IMU传播（含同步记账）2.75 ms；匹配更新 22.01 ms；地图维护 0.49 ms。
Odometry+TF publish 最大 16.1 ms；clouds组合最大 12.2 ms。稀疏采样 LiDAR 队列最老等待为0、最新 IMU 覆盖当前帧尾约2～50 ms；未见持续积压，不等于证明所有调度尖峰不存在。

## 主机时钟与 DDS

timesyncd 保持启用，没有改系统时间或校时服务。当前 boot_id `ce1aab75-85aa-4bb5-9e81-4531c874d5e1`。
本 boot 开机8.69秒恢复保存时间，39.04秒NTP初同步；发生在本轮驱动启动前。持久化旧 boot 日志不存在，不能回溯确认此前4120秒事件。
300秒外部 wall-monotonic 相对变化约 −5 μs；驱动锚点处调用偏移约 −0.037 ms 基本稳定。
`timesync_journal.txt`、`clock_basis.txt` 为证据；`dds_audit.json`、`thread_sched_30s.json` 为 QoS/调度补查。
调度汇总不能与既往214ms事件一一关联，不宣称已确认/排除DDS阻塞。
补查结果：LiDAR/IMU两端RELIABLE，Odometry的nav_adapter/观察器是BEST_EFFORT；端点发现返回depth=0并不代表本地实际队列深度为0，应以源码200/驱动256为准。未设置RMW_FASTRTPS_PUBLICATION_MODE或自定义DDS配置环境变量；未据此改QoS。30秒采样中部分驱动线程累计runqueue等待达6～8秒，说明调度竞争确实存在，但不是214ms事件的同步证据。8个CPU在线、ROS进程允许0～7；资源快照中ToDesk_Session约189%CPU、系统load约19，未关闭用户远程桌面。
额外30秒探针：IMU 5595条中5480条年龄<−0.1s，平均−112.51ms；Odometry平均−68.29ms，说明未来时间偏差持续（`future_check_30s.jsonl`）。

## Nav2 验证结果

- 实查 `/amcl` 订阅 `/initialpose`，RViz 2D Pose Estimate 发同话题。只读探针记录到用户给出的 map 位姿 x=2.42245865、y=−0.22206758、yaw=1.04751445 rad；没有猜坐标或将其改到白区。
- AMCL曾输出约(2.4321, −0.1910, 1.0498)，建立 map→odom_nav。用户说明场景不完全一致、雷达固定；该变换存在不等于定位精度已验证，且随后重启过FAST-LIO，不能把旧AMCL结果当新初始化验证。
- `/odom`、`/scan`、TF持续；扫描有限有效点通常约600。门控实际出现ready，最终 `/cmd_vel` 有旋转与线速度输出；日志对应 **恢复 spin/backup**，没有非空正常规划路径，**不计为导航成功**。
- 真实起点规划至附近自由目标(2.975, −0.975)：ComputePathToPose接受请求，结果status=6、path_poses=0。`costmap_start.json`显示起点cost254且周围大片254，目标cost138。另一个RViz目标(约3.91, −1.74)也规划失败。
- 目标取消返回return_code=0，bt_navigator确认 Goal canceled，随后最终cmd_vel持续为0。证据 `cancel_goal.txt`、`navigation_evidence.jsonl`、`plan_check.jsonl`、`nav2.log`。
- `/cmd_vel`唯一发布者nav_cmd_gate，订阅者仅本次只读观察节点（`cmd_vel_endpoints.json`），无电控消费者。
- **最终 /cmd_vel 完整验收：未通过**（缺少非空路径及正常跟随输出；时间基准B类问题亦未修复）。没有靠恢复动作冒充成功，没有放宽adapter/gate阈值，没有改地图/做PCD重定位。

## 本轮修改及保留

- 保留原有0.3m体素、IMU接收深度200及日志flush/越界保护修改。
- 驱动：原始时间/锚点/host-step/device-step稀疏日志；按frame base关联首包接收/组帧/publish；IMU真实等待和publish统计；Sampling-ready事件。IMU内部队列显式复制新增诊断字段，ROS消息定义未改。
- 删除两个分发线程的固定3秒sleep；线程本来就在Lds注册初始化后创建，等待数据仍使用原信号量。没有跳过设备Sampling检查，也未丢弃启动旧数据。
- 时间转换只改为等价有符号中间计算并加诊断，**没有做自适应重锚定/时间映射修复**。
- FAST-LIO：同帧回调到处理的steady等待、最老帧等待、去畸变传播耗时及关联stamp；保留原处理算法。
- Python：raw CDR头探针支持30秒窗口、帧trace及未来时间计数；导航只读观察、真实起点规划检查、汇总脚本。
- 编译：最终FAST-LIO完整编译通过；驱动最后一次 `build_queue_copy.log` 通过，之后重启验证，映射动态库inode核实。曾有一次诊断插入位置导致编译失败，已修正，失败日志保留但不是运行产物。
- `git diff --check`及Python语法检查通过。没有推送GitHub。
- 三份已有地图未提交修改原样保留；本轮前后hash一致：map.pgm `83986ce0…542e`，map.yaml `1a512b24…90d4`，projection.json `154484d0…38a`。没有改PCD。

## 下一步（不要把重启当修复）

1. 保留当前具有116ms偏差的驱动现场，继续采集。当前仪器本身可及时处理，但时间基准有偏差，不能宣称传感器链已正确。
2. 在下次有准备的启动前捕获设备原始时间、包类型/序号、内核接收时间及SDK回调时间（尤其首次5秒），区分首包陈旧/SDK排队与设备时间前跳。固定5秒日志漏掉的首段不能靠推测补齐。
3. 据上述证据再设计公共LiDAR/IMU锚点校准/重置和系统时间跳变处理；所有已排队数据及扫描起止必须保持同一测量时间轴，不能逐条stamp=now或任意重锚定制造相对时间畸变。
4. 214ms publish慢调用需结合线程CPU/调度、DDS可靠传输和订阅者进一步关联，不能把steady包围区间都叫DDS阻塞。
5. Nav2正常路径验证需要真实位置与可通行地图相符。本轮用户确认站在旧地图占据区，且地图禁止修改；当前约束下不通过挪假位姿、清障、放宽阈值解决。以后在有效起点重新由用户RViz给位姿，再测非空路径、正常跟随cmd_vel和取消归零。

---

# FAST-LIO /Odometry 延迟交接（2026-11-01）

## 本次范围与初始状态

Jetson / Ubuntu 22.04 / ROS 2 Humble / MID360，ROS_DOMAIN_ID=42。
开始时宿主机实际仅有 mid360_real_driver 和 laser_mapping，没有 Nav2、RViz、nav_adapter。
驱动 PID 147061，FAST-LIO PID 149775（约运行 35 分钟）。
FAST-LIO 命令行覆盖 map_en/path_en/pcd_save_en=false，scan/dense/body 点云发布仍开启。
实际参数通过 `ros2 param dump /laser_mapping` 核对；坐标系仍为 odom → livox_frame。
原有地图 map.pgm、map.yaml、projection.json 的未提交修改全部保留，没有推送 GitHub，没有启动电控。

## 证据与结论

所有表中的三路话题来自同一个探针的同一时间窗口。探针以 raw=True 接收序列化数据，
只读取 CDR 帧头 sec/nanosec，不解码点数组；采用 best effort，避免探针反压发布者。
点云时间是帧首时间，约含 100 ms 采集时长；Odometry 是帧尾时间，不能直接把两者延迟相减当计算耗时。

初始 30 秒：点云约 9.93 Hz、延迟 4120.160 s；IMU 199.96 Hz、4120.052 s；
Odometry 出现约 90.63 Hz 追赶发布，延迟 4182.504 s。
第二个 10 秒窗口：点云 9.99 Hz、4120.163 s；IMU 199.75 Hz、4120.055 s；
Odometry 8.97 Hz、4203.063 s。此时已超出用户此前的 1～3 秒现象。

驱动按完全相同参数重启后，点云约 0.112 s、IMU 0.0023 s；旧建图仍落后数千秒，之后也重启建图。
驱动 NoSync 时间转换位于 `livox_ros_driver2/src/comm/pub_handler.cpp:GetEthPacketTimestamp`：
按设备启动时间建立一次性 Unix 锚点。重启恢复说明输入时间轴有独立问题，
但本次没有足够证据区分历史系统时钟跳变、设备重启或其他锚点失效原因；未修改驱动时间算法。
系统 timesyncd 日志有开机校时记录，不能据此断言它就是本次偏移的原因。

重启后原 0.1 m 体素、45 秒基线：

| 话题 | Hz | 平均延迟 s | P95 s | 最大 s |
|---|---:|---:|---:|---:|
| /livox/lidar | 10.000 | 0.1154 | 0.1233 | 0.1386 |
| /livox/imu | 200.206 | 0.0041 | 0.0127 | 0.0468 |
| /Odometry | 10.004 | 0.0925 | 0.1225 | 0.3182 |

阶段统计：约 5000 个匹配点，更新约 60～92 ms，整帧约 65～96 ms，逼近 10 Hz 的 100 ms 预算。
Odometry 发布约 0.1～0.4 ms，点云通常约 1 ms；没有发布阻塞证据。
原运行输出长期低于输入频率支持“计算积压”的判断；重启后的短基线本身未重现长期 1～3 秒积压。

0.3 m 体素：约 1400～1500 个匹配点，更新通常约 13～28 ms，整帧通常约 16～40 ms，
LiDAR 内部队列在采样日志中为 0，IMU 与帧尾间隔通常约 10～25 ms。
精度与地图分辨率会受体素增大影响，尚未进行移动轨迹精度验证。

0.3 m 体素连续 180 秒同窗结果（其中部分窗口有编译负载）：

| 话题 | Hz | 平均延迟 s | P95 s | 最大 s |
|---|---:|---:|---:|---:|
| /livox/lidar | 10.000 | 0.1164 | 0.1267 | 0.1602 |
| /livox/imu | 199.996 | 0.0060 | 0.0144 | 0.0518 |
| /Odometry | 10.001 | 0.0459 | 0.0614 | 0.1031 |

原始汇总在 `results/`。本次窗口没有超过 0.5 s 的 Odometry，但不代表已通过数十分钟移动测试。

## 修改

- `lidar_ws/src/ros2_to_stm32/config/fast_lio_mid360_real.yaml`：surf/map 体素 0.1 → 0.3 m。
- `lidar_ws/src/fastlio/src/laserMapping.cpp`：每 5 秒输出输入延迟、内部队列、IMU 帧尾间隔、同步等待次数、预处理/整帧/更新/地图/发布耗时。
- IMU DDS 接收深度 10 → 200：200 Hz 时原来仅容纳 50 ms，短于实测建图耗时；现容纳 1 秒。
- `runtime_pos_log_enable=false` 时不再每帧写入并 flush mat_pre.txt；旧预处理统计数组增加越界保护。
- `tools/diagnostics/ros_latency.py`：可重复使用的同窗原始帧头探针。

未放宽 nav_adapter 的 0.5 s 阈值；未将旧数据的时间戳改成当前时间；未丢帧来伪装低延迟。
未更换执行器或引入并发，避免对现有全局状态造成数据竞争。

## 复测方法

```bash
cd /home/jetson/letsgo
source /opt/ros/humble/setup.bash
source lidar_ws/install/setup.bash
python3 tools/diagnostics/ros_latency.py --seconds 180
```

需在宿主 ROS 网络环境运行。受限沙箱看不到宿主进程和 DDS；本次通过受批准的宿主执行完成测量。
默认 real_lidar.launch.py 仍会打开 RViz、map/path；复现本次测试应沿用显式关闭覆盖，不能默认启动状态相同。

## 下次接续

1. 在实际移动场景连续运行 30～60 分钟，同时记录三路延迟与 `latency` 日志；验证是否随地图规模或 CPU 降频增长。
2. 继续定位输入时钟锚点为何失效，记录主机 realtime/monotonic、设备原始时间和 timestamp_type；发生前保留日志，不以反复重启当永久修复。
3. 检查 0.3 m 体素对移动轨迹、转弯、地图精度的影响，再决定是否在 0.2～0.3 m 间折中。
4. 传感器链稳定后再接 nav_adapter/Nav2，本次不验证导航或电控。
