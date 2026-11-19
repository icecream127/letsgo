# MINI 两点实机测试：从零启动与故障处理

适用目录：`~/project/letsgo`。根据本地脚本和历史诊断记录整理，日期：2026-11-17。
本文只是操作说明，编写时没有启动实机、发送动作、修改服务或安装驱动。

## 1. 本次运行的内容

- 任务：`lidar_ws/src/ros2_to_stm32/ros2_to_stm32/tasktodo_mini.py`
- 地图：同目录 `map_mini.json`
- 普通模式，身体中心依次到 `(1,0)`、`(2,0.2)`，完成后站立。
- 两点都相对同一个 A/B 赛道原点；第二点不是再前进2米。
- 保留21cm参考点补偿、5cm到点门限、8度前进角度门限。
- 不指定最终朝向，不执行桥、坡、跳跃等特殊动作。
- 沿用定位、串口、状态回传及心跳检查；不是不等回传的独立TTL脚本。

注意：A点仍然是采点时传感器位置。若传感器在A点、朝向0度，身体中心约为(-0.21,0)，到(1,0)实际约走1.21m。当前不重新定义原点。
现有UI背景没有换成两点地图；mini地图是路线配置，UI调试文字会显示mini当前目标。

## 2. 从零启动：每项使用一个终端

运行电脑必须连接雷达和TTL，并有本项目已编译的ROS 2环境。脚本自动加载环境，无需每个终端手动source。
初始保持遥控器手动模式，给马留出行走空间；初始化与接管按你现有遥控操作进行。

不要运行 `run/start.sh`、`run/start_ordered.sh` 或 `run/info_trans.sh`，避免同时启动原路线控制器。
若之前有原控制器，在其终端按Ctrl+C关闭；雷达、定位和串口不要重复启动。

### 2.1 检查连接

当前配置：电脑有线网卡192.168.1.50，MID-360为192.168.1.111。检查：

```bash
ip -br addr
ping -c 3 192.168.1.111
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/ 2>/dev/null
```

电脑有线网卡地址应匹配雷达配置；ping成功不等于点云已到达。不同网络现场先核对设备实际地址，不要盲目改Wi-Fi配置。

若实际串口是 `/dev/ttyUSB0`，设置权限：

```bash
sudo setfacl -m u:"$(id -un)":rw /dev/ttyUSB0
```

其他设备路径要替换成实际值。设备不存在时不要继续执行setfacl，先看故障部分。

### 2.2 终端一：雷达、FAST-LIO和RViz

```bash
bash ~/project/letsgo/run/lio_driver.sh
```

等待点云与定位正常更新，终端保持打开。

### 2.3 终端二：坐标转换

```bash
bash ~/project/letsgo/run/pos_trans.sh
```

### 2.4 终端三：界面和A/B踩点

```bash
bash ~/project/letsgo/run/ui.sh
```

1. 将马放在选定的A点，点击“起点”。
2. 沿希望的赛道+X方向移动一段明显距离，点击“终点”，记录B点。
3. A/B至少相距5cm；两点决定原点和方向，不决定路线长度。
4. 把马移回测试起始位置，马头朝A→B方向。

“终点”按钮只是B标定点，不是(2,0.2)任务目标。不需要点木桥、台阶等按钮。
没有新鲜里程计时踩点会被拒绝。重启FAST-LIO或pos_trans后，要重新确认并完成标定。

### 2.5 终端四：串口节点

```bash
bash ~/project/letsgo/run/serial.sh
```

终端保持打开。关闭串口调试助手、capture_ttl.py及其他占用同一串口的程序。
若设备实际为ttyUSB1，可只对本次启动覆盖端口：

```bash
bash ~/project/letsgo/run/serial.sh -p port:=/dev/ttyUSB1
```

二选一启动，不要运行两个串口节点。

### 2.6 终端五：MINI路线控制器

```bash
bash ~/project/letsgo/run/tasktodo_mini.sh
```

不需要重新编译。mini仍使用节点名info_trans和task_to_do，这不表示原路线也启动了。

完成马的初始化，等起身动作结束，确认起始位置，再用遥控器打开自动。反馈、定位和心跳条件满足后会直接开始，不需要另外点击开始。

只有之前执行过pause.sh时，才执行下面命令取消人工暂停：

```bash
bash ~/project/letsgo/run/arm.sh
```

arm.sh不会替代遥控自动开关，也不会跳过回传或定位检查。

## 3. 停止、退出与重新走一遍

优先通过遥控器切回手动接管。也可以请求软件暂停并站立：

```bash
bash ~/project/letsgo/run/pause.sh
```

这是通过串口请求站立，不是独立硬件急停。断线时不能保证停止指令送达。
暂停后，在mini终端按Ctrl+C退出；不必关闭其他节点。

只暂停再恢复会继续当前路线进度。想从第一点重新开始：先暂停、将马放回测试起始位置，在mini仍运行且没有一次性动作忙碌时执行：

```bash
bash ~/project/letsgo/run/reset_route.sh
```

然后在确认位置和遥控状态后：

```bash
bash ~/project/letsgo/run/arm.sh
```

重新启动mini进程也会从第一点开始。不要在终点原地重置后直接恢复，机器人会试图返回第一个目标。

## 4. 串口消失：BRLTTY盲文设备服务抢占

历史记录确认2026-10-27发生过：CH340生成ttyUSB0后被BRLTTY抢占，随后串口消失。
你提到的“盲写机制”可能指这个盲文设备服务；它与导航的回传检查不是同一个功能。

先确认日志：

```bash
sudo journalctl -k -b --no-pager | tail -n 100
systemctl status brltty-udev.service brltty.service --no-pager
systemctl is-enabled brltty-udev.service brltty.service
```

若日志显示brltty占用USB设备，本次开机临时停用：

```bash
sudo systemctl mask --runtime --now brltty-udev.service
```

若brltty.service本身仍在运行并占用设备，再执行：

```bash
sudo systemctl mask --runtime --now brltty.service
```

然后拔插TTL，重新检查设备路径并设置权限。`--runtime`只作用于当前开机周期。
这些命令是禁用盲文服务以释放TTL，不是禁用串口。若依赖盲文设备，不应停用该服务。

仅当确认持续需要屏蔽且不用盲文设备时，可持久屏蔽：

```bash
sudo systemctl mask --now brltty-udev.service brltty.service
```

如果系统显示已经masked，说明该服务已被屏蔽；为解决TTL抢占不需要unmask。
如果以后确实要恢复盲文服务，按之前使用方式解除屏蔽：

```bash
# 解除临时屏蔽
sudo systemctl unmask --runtime brltty-udev.service brltty.service
# 若还做过持久屏蔽，再解除持久屏蔽
sudo systemctl unmask brltty-udev.service brltty.service
```

以上恢复命令不应作为TTL排障的常规步骤。本文没有更改或确认当前系统服务状态。

## 5. 串口权限、设备名与占用

### Permission denied

```bash
ls -l /dev/ttyUSB0
sudo setfacl -m u:"$(id -un)":rw /dev/ttyUSB0
```

拔插后ACL可能失效，需要重新设置。若提示setfacl命令不存在：

```bash
sudo apt install acl
```

也可以把当前用户加入dialout组（先确认设备组是dialout）：

```bash
sudo usermod -aG dialout "$(id -un)"
```

加入后完整注销并重新登录，再重新打开终端或Codex。不要用sudo运行整套ROS节点来代替权限处理。

### setfacl提示“没有那个文件或目录”

这是设备不存在，不是权限问题：

```bash
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/ 2>/dev/null
sudo journalctl -k -b --no-pager | tail -n 100
```

核对TTL是否插好、数据线、BRLTTY抢占及实际设备名。板端USART6不代表电脑端ttyUSB6。

### 端口被占用或无法加独占锁

```bash
sudo fuser -v /dev/ttyUSB0
```

查看是哪一个程序使用设备，正常关闭对应终端/串口助手，不要直接杀掉所有Python程序。
独立TTL采集脚本和serial_node不能同时打开同一端口。

### USB可见，但没有串口设备

```bash
lsusb
lsmod | rg 'ch341|cp210x|ftdi_sio'
sudo journalctl -k -b --no-pager | tail -n 100
```

若已确认是CH340/CH341兼容设备，可尝试加载系统驱动：

```bash
sudo modprobe ch341
```

仍不识别时保留USB VID:PID和日志再判断。此前曾下载编译CH341厂商驱动，但不能据此认定已成功安装加载；不要不核对型号就写new_id或屏蔽系统驱动。

## 6. 节点都启动了，但不走：查看具体拒绝原因

在新终端加载环境并读取状态：

```bash
source ~/project/letsgo/run/environment.sh
ros2 topic echo /motor_state --once
```

重点查看reason、armed、enabled、port_open以及状态/定位数据年龄。

| 显示/现象 | 处理 |
| --- | --- |
| 人工暂停，enabled=false | 确认位置后运行arm.sh |
| 收不到马的状态 | 查上电、遥控自动、TX/RX交叉与共地、串口路径、板端实际固件回传端口 |
| 马还没初始化起身，mode=z | 按遥控流程初始化并等待完成 |
| 定位超时 | 查雷达、FAST-LIO、pos_trans和A/B标定 |
| 控制器未运行 | 启动tasktodo_mini.sh并检查其错误日志 |
| 一直切模式，不发前进 | 查看回传模式与MODE_STATUS映射是否一致，核对固件版本 |

进一步检查：

```bash
ros2 topic hz /Odometry
```

观察后Ctrl+C，再分别执行：

```bash
ros2 topic echo /pos_trans --once
ros2 topic echo /controller_alive --once
ros2 topic echo /serial_send
```

有话题但没消息时echo可能持续等待，Ctrl+C结束。/serial_send是事件请求，不一定持续高频发布。

所谓“不允许盲发”：当前serial.py要求有效板端回传、已初始化、定位及控制器心跳才允许动作。不存在一个已确认的“取消盲写禁用”命令；arm.sh只取消人工暂停。不要用移除回传检查来掩盖链路问题。

历史CyberGear源码记录是USART3，之后接线改为USART6；必须确认实机固件在USART6上同时实现接收和状态回传。更换电脑端脚本或线缆不会自动修改板子固件。
仅看到电脑write成功不等于板子收到或执行；遥控器屏幕显示自动也不能证明上位机已收到有效状态帧。

## 7. 每次节点列表不同、终端之间看不到消息

先检查各终端ROS环境是否一致：

```bash
printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION
```

之前local_preview.sh使用ROS_DOMAIN_ID=184和ROS_LOCALHOST_ONLY=1，不启动真实串口和路线控制器。实机测试关闭预览，并使用一致的环境启动全部节点。

若确认只是CLI发现缓存问题，在加载项目环境的诊断终端执行：

```bash
ros2 daemon stop
ros2 daemon start
ros2 node list
```

这不会重启实际业务节点。切换ROS_DOMAIN_ID等环境后必须重启相应业务进程，已运行节点不会跟着终端变量改变。

正常mini链路应包含info_trans、task_to_do、motor_serial、topic_trans_node以及雷达/FAST-LIO节点。mini复用原控制器节点名，不能只凭名字区分原版和mini；启动时只保留一个控制器进程。

## 8. 记录一次现场过程，方便回看

在启动mini前，用另一个终端运行：

```bash
bash ~/project/letsgo/run/record_diagnostics.sh
```

运行结束Ctrl+C停止记录，日志位于 `~/project/letsgo/diagnostics/`。它只订阅ROS，不打开TTL，不发送动作，可与串口节点共存。
保存整个记录目录，重点检查/motor_state、/motor_trace、/serial_send。

若要单独只读采集串口，先退出业务串口节点，再运行：

```bash
python3 ~/project/letsgo/run/capture_ttl.py --port /dev/ttyUSB0 --duration 20
```

不能与serial_node同时运行。这种只读采集不会验证板子执行运动能力。

## 9. 说明来源

- `DIAGNOSTICS.md`：BRLTTY抢占、只读采集、ROS记录、回传协议与固件历史。
- `FORWARD_TTL_TEST.md`：USART6接线与独立发送脚本的边界。
- `MINI_ROUTE.md`：两点任务与21cm补偿约定。
- 当前 `run/*.sh`、`serial.py`、`trans.py`、`info_transmiting.py`：实际启动参数与控制条件。

旧文档中的验证记录仅代表当时环境，不代表当前接线和实机固件已通过验证。
