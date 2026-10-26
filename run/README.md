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
| bash run/serial.sh | 串口节点：USB 转 TTL 接马主控“3”号座子（USART3），端口在 config/control.yaml |
| bash run/info_trans.sh | 任务控制器 |
| bash run/map.sh | 原项目区域地图工具 |
| bash run/pause.sh | 人工暂停并请求站立，不是急停 |
| bash run/arm.sh | 取消 pause.sh 的人工暂停 |
| bash run/reset_route.sh | 清零路线进度（一次性动作执行中不能清零），继续前确认位置 |

## 怎么开始、怎么停

总开关是遥控器的 KEY_mode：拨到自动后马才在 USART3 上回传状态，视觉收到状态就开始跑路线；
拨回手动（或遥控器按 5）后马停发状态，视觉 1 秒内自动停下并请求站立。再拨到自动就从当前路线进度继续。

1. 遥控器按 22 让马起身，bash run/start.sh 启动全部节点。
2. 在界面上踩点（A、B 两点）。
3. KEY_mode 拨到自动，马开始跑；拨回手动就停。

以下情况视觉只会暂时停住、请马站住，条件恢复后自动继续，不需要重新允许运行：
收不到马的状态、马还没起身、定位超时、控制器没运行、人工暂停（pause.sh）。
马报故障、遥控器切了模式、重新踩点都不会让视觉暂停。

默认 demo 为两点测试；完整旧障碍路线需在 control.yaml 选择 zones。
一次性动作（跳跃）的等待由上位机按 control.yaml 里的时长计时，电控不回报动作完成。
move/rename 后应重新编译，勿拷贝 build、install、log、.local、.build_sdk。

make.sh 仅用于显式指定的 CH341SER 驱动源码，不参与正常构建。
