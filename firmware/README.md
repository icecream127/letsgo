# 并马电控

上位机沿用 2026-10-26 CyberGear 普通步态和原遥控器按键帧。该版自动链路为 USART3，PB10 为 TX，PB11 为 RX，115200 8N1。板端移到其他接口的版本，按实际刷入的固件接线。上位机 USB-TTL 设备由 robot.yaml 中 motor_serial.port 指定。板端以每 80 ms 回传模式和故障，遥控器 23 启用上位机自动，遥控器 5 解除自动并请求站立。回传没有起身完成或动作确认字段。

## 断线停车

原版 kHostTimeoutMs 为 0。偶发点云匹配拒绝时 Nav2 继续工作与串口断线停车是两个独立行为。并马适配器以 100 ms 周期重发当前合法键，普通步态收到同一个键时保持相位。若上位机死机或 USB-TTL 断开，必须由电控负责在超时后站立。

parallel_horse_watchdog.patch 以 2026-10-26 CyberGear.zip 为基线，只把 kHostTimeoutMs 改成 500 ms。解压后在包含 User/ 的电控项目根目录执行检查和应用：

    git apply --check /home/zxy/letsgo/firmware/parallel_horse_watchdog.patch
    git apply /home/zxy/letsgo/firmware/parallel_horse_watchdog.patch

用并马原有工具链编译，在实车自动试跑前验证上位机关停、遥控器切手动和 USB-TTL 断线时的停车。开启超时的固件配合本次持续续发的并马入口使用。旧采点控制器只在按键变化时发运动请求，使用这份固件时持续动作会被板端超时中止。

普通多点指令见 [地图流程](../maps/README.md#4-并马普通多点导航)。
