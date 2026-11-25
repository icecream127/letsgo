# 并马电控

配套固件以 2026-10-26 CyberGear 为基线。上位机接电控 USART3，PB10 是板端 TX，接 USB-TTL RX；PB11 是板端 RX，接 USB-TTL TX，并接共地。配置为 115200 8N1。小电脑设备路径使用 `robot.yaml` 的 `motor_serial.port`，默认 `/dev/ttyUSB0`。

## 小电脑发送初始化

在解压的 CyberGear 源码根目录（含 `User/`）应用 `parallel_horse_host_init.patch`，再用原有电控工程编译和烧录：

    git apply --check /home/zxy/letsgo/firmware/parallel_horse_host_init.patch
    git apply /home/zxy/letsgo/firmware/parallel_horse_host_init.patch

遥控器拨到自动（键 23 授权），在小电脑停止占用同一串口的旧任务或导航节点后运行：

    bash run/parallel_init.sh

小电脑校验 USART3 的有效回传、无故障及初始化能力位后，只发一次键 22（十六进制 `53 16 45`）。USART3 在电控进入 Running 前回传模式 `z`，进入运行态后回传当前模式。健康状态帧的详情字节 bit 7 表示支持小电脑初始化，故障状态下的详情字节仍用于诊断。已经处于运行态时，命令直接返回。确认实车站稳后再启动导航。

## 断线停车

原版 kHostTimeoutMs 为 0。偶发点云匹配拒绝时 Nav2 继续工作与串口断线停车是两个独立行为。并马适配器以 100 ms 周期重发当前合法键，普通步态收到同一个键时保持相位。若上位机死机或 USB-TTL 断开，必须由电控负责在超时后站立。

parallel_horse_watchdog.patch 以 2026-10-26 CyberGear.zip 为基线，只把 kHostTimeoutMs 改成 500 ms。解压后在包含 User/ 的电控项目根目录执行检查和应用：

    git apply --check /home/zxy/letsgo/firmware/parallel_horse_watchdog.patch
    git apply /home/zxy/letsgo/firmware/parallel_horse_watchdog.patch

用并马原有工具链编译，在实车自动试跑前验证上位机关停、遥控器切手动和 USB-TTL 断线时的停车。开启超时的固件配合本次持续续发的并马入口使用。旧采点控制器只在按键变化时发运动请求，使用这份固件时持续动作会被板端超时中止。

普通多点指令见 [地图流程](../maps/README.md#4-并马普通多点导航)。
