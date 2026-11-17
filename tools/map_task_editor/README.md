# 离线地图任务编辑器

直接双击同目录 competition_editor.html，用 Edge、Chrome 等桌面浏览器打开。不需要 ROS、网络或重定位。页面内嵌当前 competition 地图，不会修改 PGM、YAML 或旧采点。

## 操作

1. 点击“新增障碍”，填写名称、选择通过方式。
2. 选择“设置入口”，在地图按下鼠标并拖动。按下处是位置，拖动方向是朝向；单击保留原朝向，新点默认朝向 map +x。
3. 选择“设置出口”并重复。可在右侧输入精确坐标，点击“应用坐标”。
4. 按实际底盘调整检查半径。橙色点代表位于障碍、未知区、地图外，或周边净空不足。
5. 用左侧上下箭头调整任务顺序；放大地图后通过滚动条移动视图。
6. 点击“保存任务工程”，浏览器下载 competition_tasks.json。下次点击“打开工程”恢复。工程绑定地图指纹，换图后拒绝直接导入，防止误用旧坐标。
7. 仅包含“普通导航”、所有入口出口完整并通过占地检查时，可以导出 nav2_goals.json。按任务顺序展开入口、出口，与 run/nav_goals.py 兼容。

浏览器下载通常存到 Windows 的 Downloads；工程不自动保存，关闭前请确认已下载。不要把测试点当作真实任务点。

## 坐标与限制

坐标系为 map；x/y 单位米。界面角度是度（逆时针为正），导出 yaw 为弧度。坐标转换包含 YAML origin 的平移、旋转及图像纵轴翻转。

虚线只表示访问顺序，不是规划路径。圆形占地只检查静态图，不包含动态障碍、Nav2 的代价膨胀及实际底盘形状。这里没有生成机器人实时位置，也不发送控制命令。

坡道/木桥/台阶等“专用动作”可以标注并保存在工程中，但目前动作执行器未接通，不能导出给只会发送 NavigateToPose 的程序。若某段只需要普通导航，可选择普通导航；必须依据真实底盘能力判断。

## 导出检查与执行入口

在 WSL 项目根目录先只校验下载的文件：

```bash
/usr/bin/python3 run/nav_goals.py --file /mnt/c/Users/22512/Downloads/nav2_goals.json --check-only
```

完成真实任务坐标确认、定位和底盘适配后，才使用 run/NAV_GOALS.md 的实际执行入口。本工具不负责启动导航或底盘。

## 地图更新与其他地图

页面是生成时地图的快照。更改地图后需重新生成；不要继续使用旧 HTML。

```bash
cd /home/zxy/letsgo
/usr/bin/python3 run/map_task_editor.py
# 也可加载另一份 trinary 灰度地图 YAML：
/usr/bin/python3 run/map_task_editor.py --map /绝对路径/map.yaml --output /绝对路径/editor.html
```

生成器依赖系统 Python 的 Pillow、PyYAML。HTML 自包含，可复制到 Windows 桌面打开。修改界面源码后同样重新生成。

## 验证

validation/test_map_task_editor.cjs 检查旋转地图坐标往返、已知格心、占用/未知/边界、圆形净空和工程指纹校验。浏览器验证覆盖添加任务、数值坐标编辑、工程保存恢复与 Nav2 导出。仅为离线软件验证，不代表实车导航通过。

## 启动区

点击左侧“设置启动区”，无需先创建障碍。可在地图上按下并拖动设置中心和朝向，也可在右侧填写 x/y/朝向后点击“应用坐标”。右侧“启动区长/宽”分别表示沿朝向和垂直朝向的矩形尺寸，填写后点击“应用范围”。紫色矩形随朝向旋转。

点击“保存任务工程”时，启动区以 start_area 保存，包含 pose（x/y/yaw）、size_x、size_y。旧工程没有该字段时显示为尚未设置。清除该点会清除启动中心。

启动区当前仅用于任务工程标注，不自动初始化定位、不加入 Nav2 goals，也不绘制启动区到首个入口的连线。Nav2 目标导出保持原有行为。静态净空检查只检查中心摆放姿态的圆形占地，不检查矩形范围内所有位置。

## 启动区

点击左侧“设置启动区”，在地图按下并拖动设置计划摆放位置与启动朝向，或右侧填写中心 x/y/yaw；输入长宽并点击“应用范围”。长沿朝向、宽垂直于朝向。紫色矩形保存到任务工程的 start_area；净空检查仅检查中心姿态的圆形底盘占地，不保证整个矩形内都可摆放。当前不连接到首个任务、不加入 Nav2 goals，也不发送 initialpose。旧工程缺少该字段时显示未设置。

## 两侧边界与自由选点

1. 选择障碍，点击“标两侧边界”，依次点击第一侧起点、终点、第二侧起点、终点。Esc 取消。
2. 点击“生成参考中线”。只生成青色参考线及延长线，不创建或覆盖已有入口、出口。
3. 默认勾选“沿中线选入口 / 出口”。点击“设置入口”或“设置出口”，在地图任意位置点击，点会投影到中线上；也可按下并拖动，松开位置投影为新任务点。允许超出边界端点，设置桥前/桥后的接近与退出位置。
4. 吸附时朝向自动与中线平行；两点都设置后，朝向由入口到出口的方向确定。右侧数值编辑的位置也会投影到中线，输入的 yaw 不用于自由旋转。
5. 取消勾选后，恢复任意位置和朝向编辑；不会移动已有点。重新生成中线同样保留已选点。
6. “入口出口反向”交换两点并旋转朝向。边界、参考线开关、吸附开关和任务点均随工程保存。

平行边界生成等距中线；夹角不超过 3 度时取平均方向近似，超过则拒绝。两侧须有共同纵向覆盖范围。重画边界后，旧参考线与吸附暂时关闭，点仍保留，点击生成参考中线重新启用。

工具只检查任务点的静态占地，青色参考线不是碰撞检测结果。桥外延长线没有两侧边界提供的通道保证。Nav2 导出只含目标点，不包含必须遵循的中线约束；严格沿线行驶需要额外的路径跟踪与偏离监测。

障碍出口与下一入口之间由普通导航任务发送器按成功结果依次发送目标，Nav2 负责规划过渡段；专用障碍动作交接尚未接通。

## 将保存的启动区传给当前 AMCL

本入口只提供初始位姿猜测，不执行 PCD 点云匹配，不自动开始导航。当前 AMCL 用二维地图和 /scan 定位；发送成功不代表定位正确。必须把机器人摆放在保存的启动区附近、朝向大致一致。

保存包含启动区的 competition_tasks.json 后，先离线检查：

```bash
cd /home/zxy/letsgo
/usr/bin/python3 run/initialize_start.py --project /mnt/c/Users/22512/Downloads/competition_tasks.json --check-only
```

先在一个终端按现有流程启动导航：

```bash
bash run/navigation.sh competition
```

另一个终端执行（每次启动重复同一条命令，无需重新点击起点）：

```bash
cd /home/zxy/letsgo
bash run/initialize_start.sh --project /mnt/c/Users/22512/Downloads/competition_tasks.json
```

脚本最多等待 60 秒，确认 /amcl/get_state 为 active 且 AMCL 已订阅 /initialpose，发布一次 map 坐标下 base_footprint 的 PoseWithCovarianceStamped。未就绪则退出，不发布。它不会将启动区追加成导航目标，不修改 AMCL 配置，不启动底盘。脚本与导航必须使用相同的 ROS_DOMAIN_ID、RMW_IMPLEMENTATION 和时钟；目前脚本按实时时钟使用，未配置仿真时间入口。

默认 x/y 标准差 0.5 米，yaw 标准差 15 度，仅表示初始不确定度，不代表允许误差或定位精度。可用 --xy-std 和 --yaw-std-deg 调整。矩形长宽不自动转换成协方差。

nav_adapter 的 livox_to_base_xyz / livox_to_base_yaw 仍为临时零值，实车使用前应测量。未来 PCD 定位若接收的是 livox_frame/IMU 位姿，需要结合真实安装变换转换；不能把此 AMCL 入口直接改成向任意 PCD 节点发送同一姿态。若移除 AMCL，本脚本会等待失败，需要另外实现 PCD 后端。

离线 ROS 通信验证使用模拟 AMCL 接收节点，无机器人运动节点：

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp /usr/bin/python3 validation/test_initialize_start_runtime.py
```
