# letsgo 启动脚本

在项目根目录运行，所有路径按脚本所在位置解析。

| 命令 | 用途 |
|---|---|
| bash run/build_ros2.sh | 编译项目内 SDK、驱动、FAST-LIO 和业务包 |
| bash run/start.sh --check | 检查完整业务依赖，不启动节点 |
| bash run/start.sh --dry-run | 查看将启动哪些节点 |
| bash run/start.sh | 启动全部节点，默认禁止自动运动 |
| bash run/lio_driver.sh | 只启动传感器和 RViz |
| bash run/ui.sh | 采点和状态界面 |
| bash run/pos_trans.sh | 里程计转换为踩点坐标 |
| bash run/serial.sh | UART8 对接，端口在项目 config/control.yaml |
| bash run/info_trans.sh | 任务控制器 |
| bash run/map.sh | 原项目区域地图工具 |
| bash run/arm.sh | 初始化、踩点和现场确认后，输入 RUN 允许运动 |
| bash run/pause.sh | 暂停任务并尽力请求站立，不是急停 |
| bash run/reset_route.sh | 暂停后清零路线进度，重新运行前确认位置 |

默认 demo 为两点测试；完整旧障碍路线需在 control.yaml 选择 zones。
请阅读根目录 README.md。一次性动作等待由上位机计时，无固件动作完成确认。
控制器重启会请求暂停，串口恢复不会自动允许运动。
move/rename 后应重新编译，勿拷贝 build、install、log、.local、.build_sdk。

make.sh 仅用于显式指定的 CH341SER 驱动源码，不参与正常构建。
