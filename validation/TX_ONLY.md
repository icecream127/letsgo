# USART3 单向发送

control.yaml 的 motor_serial.tx_only=true 启用单向发送。小电脑仍打开 /dev/ttyUSB0，115200 8N1，发送 53 key 45。不解析反馈，不确认电控初始化、故障、模式、自动授权或动作执行结果。

模式变化在同一次串口处理回调中依次发送站立、模式键和动作键，不插入模式切换等待。一次性动作仍使用 action_durations 加 action_margin 计时，不是完成反馈。不重复发送相同的持续运动请求。

/motor_state 的 connected、mode、fault、motor、detail 在单向模式为 null；serial_open 仅表示本地串口已打开；last_tx 只表示本地写入成功。busy_source=host_timer，action_completion_feedback=false。

流程：启动节点，遥控器初始化，完成 A/B 踩点，核对起跑位置和路线，遥控器按23授权，再运行 arm.sh。必须确认遥控器固件能够发送23。路线审核、定位超时、控制器心跳、人工启动及串口异常检查继续保留；地图审核未通过仍不能解锁。

遥控器接管后必须暂停上位机并核对路线进度，禁止直接再次按23续跑。上位机无法感知遥控器关掉自动权限或改变模式。pause.sh 是站立请求，不是可靠急停；忙碌保护期可能延后发送。当前电控断线保护关闭，USB断开、程序退出或系统故障均不保证机器人停止。保持独立急停和现场监护。

未修改电控固件。未进行真机验证。
