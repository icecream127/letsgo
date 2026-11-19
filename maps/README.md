# 建图、采点与导航简明流程

以下命令适用于原生 Ubuntu 22.04 和 WSL Ubuntu 22.04，进入项目目录 `/home/zxy/letsgo` 后执行。用 `trial` 代表新地图名，各步骤使用同一个名字。首次部署先执行 `bash run/build_ros2.sh --gicp`。已有配套地图时从采点开始。

## 1. 采集并保存点云

终端一启动建图。让机器人在开阔平地静止初始化，再扫描场地。

```bash
bash run/mapping.sh trial
```

保持建图运行，终端二保存。看到保存成功后关闭终端一。

```bash
bash run/save_map.sh trial
```

点云保存为 `maps/trial/cloud.pcd`。

## 2. 生成二维地图

```bash
bash run/map_to_2d.sh trial --auto-ground
```

生成 `map.pgm`、`map.yaml` 和 `projection.json`。需要 GIMP 修图时，按下方说明导入后再采点。

## 3. 在地图上采点

```bash
bash run/map_task_editor.sh trial
```

命令会用默认浏览器打开本地采点界面，终端中也会打印地址。采点工具需要 `python3-yaml` 和 `python3-pil`。已有比赛地图可直接运行 `bash run/map_task_editor.sh`。标好启动区的摆放位置和朝向，再标各障碍入口、出口并排序。已有工程会自动载入，也可用「打开工程」导入旧 JSON。

点击「保存任务工程」，写入 `maps/trial/competition_tasks.json`。全部任务为普通导航且点位通过占地检查时，同时写入 `maps/trial/nav2_goals.json`。如果新工程不可导出，保存时会清除旧目标文件。保存完成后可按 Ctrl+C 关掉采点服务。

## 4. 启动重定位和 Nav2

把机器人放到标好的启动区。首次使用前，在 `robot.yaml` 填好实测的雷达到底盘安装外参。输入 IMU 在固定 PCD 坐标中的初始 Z，12 厘米启动台见下方说明。

```bash
read -rp 'IMU 初始 Z（米）: ' imu_z
bash run/navigation_gicp.sh trial --imu-z "$imu_z"
```

保持静止，确认 `/gicp/status` 持续出现 `ACCEPTED`，并在 RViz 核对点云对齐、底盘位置和朝向。可在启动命令末尾加 `--check-only`，先查看计算出的 IMU 初猜。

## 5. 发送普通导航目标

定位确认后，另开终端：

```bash
bash run/nav_goals.sh --map trial
```

使用比赛地图时可直接运行 `bash run/nav_goals.sh`。前一目标到达后，Nav2 从当前位置规划到下一目标。当前导航实现到 `/cmd_vel` 输出，实车执行还需底盘速度接口，专用越障还需动作交接。

## 按现场需要调整

- **GIMP 修图**。打开 `map.pgm`，保留尺寸和像素位置，用铅笔修改。近白 `FEFEFE` 为可通行、黑 `000000` 为障碍、灰 `CDCDCD` 为未知。在桌面另存 `map_edited.pgm`，PGM 格式选「原始（二进制）」。导入后重新运行采点命令。以下示例用 WSL 的 Windows 桌面路径，原生 Ubuntu 请替换为实际文件路径。

  ```bash
  /usr/bin/python3 run/import_map_image.py trial /mnt/c/Users/22512/Desktop/map_edited.pgm
  ```

- **墙外散点或灰边太大**。新建图时可加 `--radius-outlier 0.10` 去散点，默认要求 3 个邻点，需安装 `python3-scipy`。可用 `--bounds XMIN YMIN XMAX YMAX` 限定投影范围，脚本会配套更新 YAML。裁剪只改变栅格左下角的位置，固定 PCD 和任务点的 `map` 坐标仍按原坐标计算。更新地图后重新生成编辑器，并核对、保存与新图配套的任务工程。
- **12 厘米启动台**。`--imu-z = 平台顶面在 PCD 中的 Z + IMU 离平台顶面的垂直高度`。沿用同一份 PCD，且站姿相同，平地初猜加 `0.12 m` 即可，例如 `-0.20 → -0.08 m`。台面水平时可沿用默认的横滚和俯仰初猜，二维地图原点和安装外参无需因台高调整。当前地形投影默认单级上限为 10 厘米，12 厘米垂直边缘可能被判为不连通。有坡道就核对平台至坡脚的路线，只有垂直边缘时需先确认底盘下台能力和通过方式。

原有详细说明见 [competition 制图说明](competition/README.md)、[采点编辑器](../tools/map_task_editor/README.md) 和 [GICP 导航](../run/GICP_NAVIGATION.md)。