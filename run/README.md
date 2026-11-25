# letsgo 启动脚本

在项目根目录运行，所有路径按脚本所在位置解析。

| 命令 | 用途 |
|---|---|
| bash run/build_ros2.sh | 编译项目内 SDK、驱动、FAST-LIO 和业务包 |
| bash run/start.sh --check | 检查完整业务依赖，不启动节点 |
| bash run/start.sh --dry-run | 查看将启动哪些节点 |
| bash run/start.sh | 启动全部节点 |
| bash run/lio_driver.sh | 只启动传感器和 RViz |
| bash run/ui.sh | 采点和状态界面 |
| bash run/pos_trans.sh | 里程计转换为踩点坐标 |
| bash run/serial.sh | 串口节点，USB-TTL 接电控 USART3，主机设备路径在 robot.yaml |
| bash run/parallel_init.sh | 旧按键协议固件的上位机初始化 |
| bash run/parallel_forward_test.sh | H743 PC Link 自启动并前进 0.5 米后收步 |
| bash run/info_trans.sh | 任务控制器 |
| bash run/map.sh | 原项目区域地图工具 |
| bash run/pause.sh | 人工暂停并请求站立，不是急停 |
| bash run/arm.sh | 取消 pause.sh 的人工暂停 |
| bash run/reset_route.sh | 清零路线进度（一次性动作执行中不能清零），继续前确认位置 |

## 怎么开始、怎么停

USART3 固件在 KEY_mode 拨到自动后回传状态，视觉收到状态且定位、控制器就绪后开始跑路线；
拨回手动（或遥控器按 5）后马停发状态，视觉 1 秒内自动停下并请求站立。再拨到自动就从当前路线进度继续。

1. 遥控器按 22 让马起身，bash run/start.sh 启动全部节点。
2. 在界面上踩点（A、B 两点）。
3. KEY_mode 拨到自动，马开始跑；拨回手动就停。

以下情况视觉只会暂时停住、请马站住，条件恢复后自动继续，不需要重新允许运行：
收不到马的状态、马还没起身、定位超时、控制器没运行、人工暂停（pause.sh）。
马报故障、遥控器切了模式、重新踩点都不会让视觉暂停。

默认 demo 为两点测试；完整旧障碍路线需在 robot.yaml 选择 zones。
一次性动作（跳跃）的等待由上位机按 robot.yaml 里的时长计时，电控不回报动作完成。
move/rename 后应重新编译，勿拷贝 build、install、log、.local、.build_sdk。

make.sh 仅用于显式指定的 CH341SER 驱动源码，不参与正常构建。

## H743 自启动和 0.5 米通信测试

使用启用了 PC Link 的 H743 固件。按固件要求摆好起立姿态，保持遥控器连接并停在状态页。等待电控到 READY，无需按遥控启用键。关闭占用 USB-TTL 的导航或串口进程，并清空前方测试路段。

在小电脑的两个终端分别执行。如果已有 FAST-LIO 在发布 /Odometry，直接运行第二条。

1. `bash run/lio_driver.sh show_rviz:=false`
2. `bash run/parallel_forward_test.sh`

脚本确认 FAST-LIO 和 PC Link 回传后发送 T1，由电控自行预检、起立并进入 TROT。首次收到 RUN / WALKING 时记录起点，水平位移达到 0.50 米时发送同会话 Z1。收到 RUN / STOPPED 才确认完成。测试超时、里程计中断或 Ctrl+C 时，脚本也会尝试发送 Z1。串口失联时由固件的 500 ms 超时处理，遥控器 KEY7 可人工停止。实际停车距离请现场核对。

电控接线及预设见 [H743 USART3 说明](https://github.com/alpha123ad/homework/blob/40a449a7a99b2ad27a9b21fc36d50ed0240aafd5/cybergear_h743/README.md#usart3-小电脑动作控制)。小电脑设备路径和测试参数在 `lidar_ws/src/ros2_to_stm32/config/robot.yaml` 的 `motor_serial`。

## Nav2 坐标任务

并马普通多点使用 `auto_nav.sh --parallel`，采点、校验和启动步骤见 [简明流程](../maps/README.md#4-并马普通多点导航)。
已启动导航时，可用 nav_goals.sh 单独下发目标，见 [NAV_GOALS.md](NAV_GOALS.md)。


## 离线地图任务编辑器

使用说明：../tools/map_task_editor/README.md；生成入口：map_task_editor.py。


PCD 重定位入口见 [GICP_NAVIGATION.md](GICP_NAVIGATION.md)。
