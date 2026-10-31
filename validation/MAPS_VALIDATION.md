# 地图接入验证（2026-10-29）

修改目标：`/home/zxy/letsgo`，基于 `db8b420`，未提交或推送远端。

## 已完成

- colcon 构建 `fast_lio`、`ros2_to_stm32` 成功。保留上游 CMake/Boost 警告，没有编译错误。
- `validation/test_maps.py`：5 项通过。覆盖 ASCII/binary PCD（含额外字段）、无效点与截断文件、投影方向、障碍优先、未知空间、无地面拒绝、网格尺寸限制、目录校验、覆盖保护和源图更新校验。
- `validation/test_map_runtime.py`：2 项通过。提取实际 C++ 保存函数用 PCL 编译执行，验证空点云拒绝、创建目录、binary PCD 写入和错误路径失败；该文件投影后由官方 Nav2 map_server 加载、激活，晚加入订阅者收到 transient-local `/map`，格子值为预期的 `[100, -1, -1, 0]`。
- 启动已构建的 FAST-LIO 节点（没有驱动或传感器输入），真实 `/map_save` 返回空地图失败，未产生伪空地图文件。
- 新旧 launch 的 `--show-args` 检查通过；新增 shell 脚本 `bash -n` 和 `git diff --check` 通过。

最终联合测试：**7 passed**，隔离 `ROS_DOMAIN_ID=189`、`ROS_LOCALHOST_ONLY=1`、Cyclone DDS。早期默认 Fast DDS 测试出现跨进程服务请求超时（不是地图解析失败）；没有以这些失败运行作为通过证据，没有改全局 RMW 配置。复现命令见 maps/README.md。

## 边界

- 没有连接实际 Mid360，也没有采集真实赛道地图。
- 合成验证地图留在 pytest 临时目录，没有放入正式 maps 场地目录。
- 仅接入 Nav2 map_server 和 lifecycle manager，不代表 AMCL、map → odom、导航目标或 cmd_vel 已集成。
- 投影是平地高度筛选，空闲只来自地面观测；需要现场确认地面 Z、高度范围、地图连通性和机器人尺寸。没有回环优化、自动地面拟合或坡道/台阶可通行判断。
- 没有启动电控、串口和旧任务控制器；测试启动的节点已关闭。
