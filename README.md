# letsgo：完整上位机独立源码包

## 2026-10-26 电控配套版（以下历史说明以本节为准）

- 当前接线改为板端串口6，上位机发送 `53 key 45` 原始三字节，115200、8N1、无流控。电脑端设备仍按实际配置（默认 `/dev/ttyUSB0`）。电控需提供适配串口6的固件；原1026固件配置的是USART3。
- 当前上位机仍要求自动模式下回传原 `A5 5A` 状态协议（原周期80ms）。串口6新版固件的回传行为待核对，发送键值表一致不代表回传也已确认兼容。
- `route_profile: zones`；使用 Python 包目录的 `map_zones_config.json` 学长主地图，不是地图2。障碍优先，普通区域按 func_name 转交对应障碍处理函数。
- 八段航点对齐原始 unitree 版本，台阶第三段使用台阶模式小跳2。协议适配、进度管理和非阻塞动作等待仍为改造实现，不等于原代码逐行恢复。
- 普通动作每请求连发4帧，普通模式切换2帧，路线一次性动作及其模式切换各1帧。相同动作仍去重。
- UI 区域和坐标只由 /pos_trans 更新，原有布局及格式保留。
- 不再使用地图哈希审核和导航无进展超时。默认 enabled=true，反馈、定位及控制器条件就绪即可自动运行，条件恢复会自动继续，不保证必须先运行 arm.sh。
- 拨回手动会停止电控回传，上位机等待状态超时后转为不活动。站立请求不是独立急停；板端通信超时停止仍关闭；动作等待仍是本地计时，不是完成反馈。
- 25项离线测试通过，未验证真机完整赛道。旧 validation/CHAIN_REVIEW.md、validation/TX_ONLY.md 是历史记录，不代表本版配置。

## 2026-10-23 开发核查版（优先于下方旧版说明）

默认已改为地图2 + zones，地图位于 lidar_ws/src/ros2_to_stm32/config/map_zones_config_2.json，数值未改。
本轮只改上位机，电控未改。完整链路问题、修复、验证边界见 validation/CHAIN_REVIEW.md。
当前旧航点有18个固定目标落在全部调度区域之外，不能宣称恢复完赛。
新增路线审核状态，reviewed_map_sha256 为空时 arm 拒绝运行；不可只填hash绕过已知路线问题。
bash run/check_route.sh --check 可离线查看问题，不启动硬件。
桌面旧 letsgo.tar.gz 不含本轮开发改动；下面关于默认demo及旧地图路径的说明仅为上一版记录。

本版本保留学长项目的采点界面、坐标转换、地图区域及障碍路线，使用你的 FAST-LIO、Livox 驱动和项目内 Livox-SDK2。
串口及动作映射适配 2026-10-21 提供的 CyberGear-1；不修改电控固件，不主动发送 21/22 初始化或趴下命令。

## 先看这几条

- 遥控器完成初始化后再采 A/B 点，最后由操作员允许自动运行。
- 启动节点默认不下发运动；串口连接或恢复不自动启用任务。
- 默认路线是 demo：顺序到 (1, 0, 0°)、(1, -1, 0°)，不是完整赛道。
- 完整旧障碍路线保留，需把 control.yaml 中 route_profile 改为 zones。旧赛道坐标和动作选择必须现场核对，不能宣称已经验证完赛。
- 按键 5 只是站立，不是急停；固件断线后可能继续最后动作。独立急停必须就绪。
- 一次性动作按源码时长加余量等待，无动作完成/成功反馈，不自动重发。
- 暂停发生在一次性动作过程中时，需人工检查结果，并清零路线进度后才能重新允许自动运行。
- 遥控器同模式的运动按键不会体现在状态帧中，上位机无法识别这种人工接管。手动接管前先暂停自动任务。

## 目录与参数

所有运行参数在本项目内，不读取其他工作空间：
- lidar_ws/src/ros2_to_stm32/config/control.yaml：串口设备、超时、动作时长和路线选择。
- 同目录 fast_lio_mid360_real.yaml：沿用此前调好的雷达/IMU参数，累计地图开启，PCD保存关闭。
- 同目录 mid360_real.json：主机 192.168.1.50、雷达 192.168.1.111，部署时按实际网络核对。
- 同目录 livox_driver.yaml 和 sensor.rviz：保留上一版配置。
- lidar_ws/src/ros2_to_stm32/ros2_to_stm32/map_zones_config.json：旧赛道区域坐标。
- 同目录 task_to_do_unitree.py：当前入口实际使用的障碍路线和导航。
- validation/：离线验证与此前传感器回放基线说明。

串口默认 /dev/ttyUSB0，不自动选择第一个设备。建议在 control.yaml 改成实际 /dev/serial/by-id/... 稳定路径。
源码包中其他 FAST-LIO 示例参数和旧控制器变体为原项目保留文件，正式入口不使用它们。

## 在小电脑部署

在 Ubuntu 22.04 / ROS 2 Humble 中解压后，进入 letsgo：
```bash
tar -xzf letsgo.tar.gz
cd letsgo
rosdep install --from-paths lidar_ws/src --ignore-src -r -y --rosdistro humble
bash run/build_ros2.sh
bash run/start.sh --check
```

需要 ROS 2 Humble、colcon、rosdep、CMake、C++ 编译器、pyserial、matplotlib、tkinter。
图形一键启动还需要 gnome-terminal。SDK 在项目内编译安装，无需往系统安装 SDK。
本版坐标转换直接计算四元数航向，不再依赖 tf_transformations。
内存紧张时：BUILD_JOBS=1 bash run/build_ros2.sh。

不要复制其他电脑的 build/install/log/.local/.build_sdk。换位置或改外层目录名后重新构建。
脚本按自身位置找工程，并检查 ROS 包来自当前工程。不要同时启动旧版上位机。

## 启动流程

只看点云和里程计，不启用串口：
```bash
bash run/lio_driver.sh
```

不打开 RViz：
```bash
bash run/lio_driver.sh show_rviz:=false
```

完整节点（需要桌面会话）：
```bash
bash run/start.sh
```

通过 SSH 无桌面时，分别在终端运行传感器、pos_trans.sh、serial.sh、info_trans.sh；
采点 UI 的 ui.sh 仍需可用图形会话。不要将启动节点误认为已允许自动运行。

1. 使用遥控器完成初始化，人工确认姿态正常。
2. 在界面依次采 A 点、B 点。A/B 间距至少 5cm；实际采点应留足距离并沿场地正方向。
3. 核对 control.yaml 路线和串口、现场位置及急停。
4. 运行下列命令并输入 RUN，成功响应后才允许运动：

```bash
bash run/arm.sh
```

暂停任务（非急停）：
```bash
bash run/pause.sh
```

暂停后清零路线进度：
```bash
bash run/reset_route.sh
```

清零会从路线起点重新执行，必须先人工确认当前位置和姿态适合重跑；不提供自动跳过失败障碍。
脚本调用结果以 success/message 为准。暂停/异常只尽力请求站立，不能保证固件已停。

## 控制适配内容

- 上行使用 53 / 原始键值 / 45 三字节，不附加回车换行。
- 下行校验八字节状态帧 CRC，发布 /motor_state（JSON）。
- 上位机内部 /serial_send 改为 [动作键, 1, 模式键, 请求序号]，不是板端协议。
- 请求序号只在上位机内部用于去重与本地发送跟踪，未添加到电控协议。
- /motor_state 的 last_tx 只表示本地串口写入成功；busy 是上位机计时保护，不是电控真实忙碌反馈。
- 模式改变先发送模式键、等待状态匹配，再发送动作；不重复发送跳跃。
- 连续动作按变化发送；到点/到角度下发站立，保留定位闭环。
- 原同一次回调连续发两个目标的问题已改成顺序推进；身后目标不再误判到达。
- 障碍等待改为非阻塞计时，等待期间继续接收定位和故障。
- 台阶第三次动作对齐为普通模式 9 + 按键 6；前两次为台阶模式 18 + 6/7。
- 保留其余旧路线的具体动作选择，包括大斜坡处理函数内既有木桥B跳跃。它们是旧赛道策略，不根据区域名称擅自重写。
- 定位、控制器心跳、状态超时，故障、z 状态或重新踩点会锁定暂停；恢复后不自动续跑。
- 没有新增固件级看门狗，不能解决小电脑掉电、进程被强杀时的硬件停止问题。

## 传感器约束

保留 /Odometry、/cloud_registered、/cloud_registered_body、/path、累计 /Laser_map。
TF 为 odom → livox_frame，PCD保存关闭。
/Laser_map 保留你的 FAST-LIO 实现：约 1Hz 追加当前帧，不是每一帧的无遗漏拼接，内存会持续增长。
本次不改 FAST-LIO 算法及传感器参数数值。

## 离线测试

测试不打开实际 TTL、不连接实物；虚拟串口测试只操作系统分配的 PTY。
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=184 ROS_LOCALHOST_ONLY=1
cd lidar_ws/src/ros2_to_stm32
python3 -m pytest test/test_motor_adaptation.py test/test_motor_integration.py -q
```

实机首次验证顺序：只看状态 → 初始化与踩点 → 空旷区域验证短距离行走/转向/暂停 → 单障碍 → 完整路线。
离线通过不等于已验证实机方向、动作成功或整场完赛。
