# Nav2 坐标目标与多点任务

本入口使用 `/navigate_to_pose` Action，与具体定位算法无关。当前先沿用现有导航定位；后续接入 PCD 定位无需更改任务文件。它不启动传感器、定位或底盘串口，不替代 Nav2 控制器。

## 1. 启动导航

在 WSL 的项目根目录：

```bash
cd /home/zxy/letsgo
bash run/navigation.sh competition
```

当前仍需按现有流程完成定位初始化，确认地图与激光对齐、定位有效。以后改用 PCD 定位时，需另外适配启动项与定位门控。没有有效定位时，发送目标不意味着机器人可以行驶。

## 2. 单目标

在第二个终端：

```bash
cd /home/zxy/letsgo
bash run/nav_goals.sh --goal 1.2 -2.95 0 --check-only
bash run/nav_goals.sh --goal 1.2 -2.95 0 --timeout 180
```

坐标均属于固定 `map` 坐标系，x、y 单位米，yaw 单位弧度，绕 Z 轴逆时针为正。示例坐标仅用于说明，实际执行前必须检查目标可达性及机器人位置。`--check-only` 只检查输入，不检查地图通行性，也不发送目标。目标是否到达由 Nav2 的 goal checker 容差判定，并非数学上的零误差。

## 3. 多目标任务

自行创建 `mission.json`，替换成现场确认的任务坐标：

```json
{
  "frame_id": "map",
  "goals": [
    {"name": "A", "x": 1.2, "y": -2.95, "yaw": 0.0},
    {"name": "C", "x": 4.925, "y": -4.075, "yaw": 1.5708}
  ]
}
```

```bash
bash run/nav_goals.sh --file mission.json --check-only
bash run/nav_goals.sh --file mission.json --timeout 180
```

每个点单独发送 NavigateToPose。仅返回 SUCCEEDED 后进入下一点；拒绝、失败、取消或超时停止，不自动跳点或重试。超时按每点计算，使用实际经过时间。Ctrl+C 请求取消当前目标；请等待取消处理结束，若提示取消未确认，应检查导航状态并按需停止底盘。正常全部完成退出码为 0，其余为非零。不要同时运行多个任务客户端或在 RViz 发送新目标，以免互相抢占。

任务点表达目的地，不需要沿行驶路径密集采点。Nav2 根据二维地图和实时障碍自行规划路径。地图重新建图、更换原点或旋转后，必须重新核对所有任务点。

## 4. 验证边界

```bash
/usr/bin/python3 -m unittest validation.test_nav_goals -v
```

测试覆盖任务顺序、失败中止和输入校验。该测试不驱动底盘，也不证明定位、避障或实车行驶成功。现场依次验证：RViz 单目标、客户端单目标、两个任务点；检查到达朝向、失败停止和 Ctrl+C 取消，再扩展任务。现有 navigation 入口不自动启动串口，底盘联调沿用项目现有流程。

启动区初始化：使用 initialize_start.sh --project <任务工程.json>，详见 ../tools/map_task_editor/README.md。它只发送 AMCL 初始猜测，不确认定位成功。

