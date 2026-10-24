"""

                               _(\_/)
                             ,((((^`\
                             ((((  (6 \
                          ,((((( ,    \
       ,,,_              ,(((((  /"._  ,`,
      ((((\\ ,...       ,((((   /    `-.-'
      )))  ;'    `"'"'""((((   (      
    (((  /            (((      \
      )) |                      |
     ((  |        .       '     |
     ))  \     _ '      `t   ,.')
     (   |   y;- -,-""'"-.\   \/
    )   / ./  ) /         `\  \
        |./   ( (           / /'
        ||     \\          //'|
        ||      \\       _//'||
        ||       ))     |_/  ||
        \_\     |_/          ||
        `'"                  \_\
                             `'"
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
# 导入 Matplotlib 相关库 (为你提供的类补充的导入)
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from .zone_manager import ZoneManager, zone_config_path 
import math
import matplotlib
# 优先使用 Windows 的黑体，如果没有则使用 Linux 的文泉驿微米黑/正黑
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Noto Sans CJK SC']
matplotlib.rcParams['axes.unicode_minus'] = False  # 正常显示负号
# ----------------- 你提供的绘图类 -----------------
class draw_node():
    def __init__(self):
        self.zones = {}
        self.zone = {}
        self.zon = {}

        # 开启交互模式
        plt.ion()

        self.fig, self.ax = plt.subplots(figsize=(6, 8))
        self.ax.set_xlabel("Y轴 (向右) [m]")
        self.ax.set_ylabel("X轴 (向前) [m]")
        self.ax.grid(True, linestyle='--', alpha=0.6)

        self.robot_width = 0.4
        self.robot_height = 0.45

        self.robot_point, = self.ax.plot([], [], 'ro', markersize=4)
        self.robot_rect = patches.Rectangle(
            (0, 0), self.robot_width, self.robot_height,
            linewidth=1.5, edgecolor='r', facecolor='none'
        )
        self.ax.add_patch(self.robot_rect)
        self.static_map_drawn = False

    def _draw_static_elements(self, zones, zone, zon):
        start_point_added_to_legend = False
        for name, data in zones.items():
            if "x_min" in data and "x_max" in data:
                plot_width = data["y_max"] - data["y_min"]
                plot_height = data["x_max"] - data["x_min"]
                rect = patches.Rectangle((data["y_min"], data["x_min"]),
                                         plot_width, plot_height,
                                         linewidth=2, edgecolor='blue', facecolor='lightblue', alpha=0.5)
                self.ax.add_patch(rect)

            if "begin_x" in data and "begin_y" in data:
                if not start_point_added_to_legend:
                    start_point_added_to_legend = True
                else:
                    self.ax.plot([data["begin_y"]], [data["begin_x"]], marker='*', color='darkorange', markersize=5,
                                 linestyle='None')

        for name, data in zone.items():
            if "x_min" in data and "x_max" in data:
                plot_width = data["y_max"] - data["y_min"]
                plot_height = data["x_max"] - data["x_min"]
                rect = patches.Rectangle((data["y_min"], data["x_min"]),
                                         plot_width, plot_height,
                                         linewidth=2, edgecolor='green', facecolor='lightgreen', alpha=0.5)
                self.ax.add_patch(rect)

        for name, data in zon.items():
            if "x_min" in data and "x_max" in data:
                plot_width = data["y_max"] - data["y_min"]
                plot_height = data["x_max"] - data["x_min"]
                rect = patches.Rectangle((data["y_min"], data["x_min"]),
                                         plot_width, plot_height,
                                         linewidth=2, edgecolor='green', facecolor='lightgreen', alpha=0.5)
                self.ax.add_patch(rect)

        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlim(-5, 8)
        self.ax.set_ylim(-10, 5)
        self.static_map_drawn = True

    def update_map(self, robot_x, robot_y, yaw, zones, zone, zon):
        if not self.static_map_drawn:
            self._draw_static_elements(zones, zone, zon)

        self.robot_point.set_data([robot_y], [robot_x])

        bottom_left_coord = (robot_y - self.robot_width / 2, robot_x - self.robot_height)
        self.robot_rect.set_xy(bottom_left_coord)
        
        # 注意：这里假设传入的 yaw 是角度(degrees)。如果是弧度，请在外部或这里进行转换
        plot_yaw_deg = -yaw
        t = transforms.Affine2D().rotate_deg_around(robot_y, robot_x, plot_yaw_deg) + self.ax.transData
        self.robot_rect.set_transform(t)

        self.ax.set_title(f"场地地图 | X(向前): {robot_x:.2f}m, Y(向右): {robot_y:.2f}m")

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


# ----------------- ROS 2 节点类 -----------------
class RobotMapVisualizerNode(Node):
    def __init__(self):
        super().__init__('robot_map_visualizer')

        # 1. 实例化绘图类
        self.drawer = draw_node()

        # 2. 定义静态地图区域数据 (请替换为你自己的真实数据)
       # 1. 实例化管理器
        self.zm = ZoneManager(zone_config_path())
        
        # 2. 读取数据
        all_data = self.zm.load_data()
        
        # 3. 获取 zones 数据，并将 func_name 还原为真正的函数引用
        self.zones = self.zm.bind_functions(self, all_data["zones"])
        self.zone = all_data["zone"]
        self.zon = all_data["zon"]
        
        self.get_logger().info("成功加载并解析区域坐标数据！")
        # 3. 创建订阅者
        self.pos_trans_subscription = self.create_subscription(
            Float32MultiArray,
            '/pos_trans',
            self.pos_trans_callback,
            20
        )
        self.get_logger().info("地图可视化节点已启动，正在等待 /pos_trans 数据...")
    def pos_trans_callback(self, msg):
        # 确保收到的数据至少有3个元素 (X, Y, Yaw)
        if len(msg.data) >= 3:
            robot_x = msg.data[0]
            robot_y = msg.data[1]
            yaw = -msg.data[2]

            # 【注意】ROS 2 中通常发送的是弧度 (Radians)
            # 如果你的 /pos_trans 发送的是弧度，但你的 draw_node 需要角度 (degrees)
            # 请取消注释下面这行代码进行转换：
            # yaw = math.degrees(yaw) 

            # 调用绘图类更新地图
            self.drawer.update_map(robot_x, robot_y, yaw, self.zones, self.zone, self.zon)
            
            # 为了防止某些 Linux 桌面环境下 GUI 卡顿，可以加一个极短的 pause 辅助刷新
            plt.pause(0.001)
        else:
            self.get_logger().warn("收到的 /pos_trans 数据长度不足 3 (需要 X, Y, Yaw)")

def main(args=None):
    rclpy.init(args=args)
    node = RobotMapVisualizerNode()
    
    try:
        # spin 会持续监听话题并触发回调
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号，关闭节点...")
    finally:
        # 清理工作
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all') # 关闭 Matplotlib 窗口

if __name__ == '__main__':
    main()
