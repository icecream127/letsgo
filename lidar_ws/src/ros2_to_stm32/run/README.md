# 本项目启动入口

请阅读项目根目录 README.md。

- 构建：bash run/build_ros2.sh
- 只启动点云和里程计：bash run/lio_driver.sh
- 完整业务预检：bash run/start.sh --check
- run/start.sh 会启动旧控制器和串口，本阶段不使用。

包内 run 脚本会转发到项目顶层 run；不再使用旧电脑绝对路径。
