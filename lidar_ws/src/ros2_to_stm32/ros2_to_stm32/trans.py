# 导入ROS2相关库
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import TransformStamped
import math
import time
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from .zone_manager import ZoneManager, zone_config_path 
class TopicRenamer(Node):
    def __init__(self):
        super().__init__('topic_trans_node')
        self.x1 = 0.0
        self.y1 = 0.0
        self.x2 = 0.0
        self.y2 = 0.0
        self.x_p = 0.0
        self.y_p = 0.0
        self.z_p = 0.0
        self.x_trans = 0.0
        self.y_trans = 0.0
        self.first_captured = False
        self.second_captured = False
        self.last_odometry_time = float('-inf')
        self.calibration_publisher = self.create_publisher(Bool, '/calibration_ready', 1)

        self.zm = ZoneManager(zone_config_path())

        self.task_param_subscription = self.create_subscription(
            Float32MultiArray,
            '/task_params',
            self.task_param_callback,
            10)
        self.subscription = self.create_subscription(
            Odometry,  
            '/Odometry',
            self.Odometry_callback,
            10  
        )

        self.publisher = self.create_publisher(Float32MultiArray, '/pos_trans', 20)
    def update_zone_data(self, zone_name, new_x, new_y):
        """
        如果你需要在节点运行中修改数据并保存
        """
        # 重新加载最新数据（防止覆盖别人刚写的）
        current_data = self.zm.load_data()
        
        # 修改数据
        current_data["zones"][zone_name]["begin_x"] = new_x
        current_data["zones"][zone_name]["begin_y"] = new_y
        
        # 保存回文件
        self.zm.save_data(current_data)
        self.get_logger().info(f"{zone_name} 数据已更新并保存至文件。")
    def task_param_callback(self, msg):
        if not msg.data:
            return
        if msg.data[0] in (1.0, 2.0) and time.monotonic() - self.last_odometry_time > 1.0:
            self.get_logger().error('没有新鲜里程计，拒绝踩点')
            return
        if msg.data[0] == 1.0:
           self.x1 = self.x_p
           self.y1 = self.y_p
           self.first_captured = True
           self.second_captured = False
        elif msg.data[0] == 2.0:
            if not self.first_captured:
                self.get_logger().error('请先采 A 点')
                return
            self.x2 = self.x_p
            self.y2 = self.y_p
            self.second_captured = math.hypot(self.x2 - self.x1, self.y2 - self.y1) >= 0.05
            if not self.second_captured:
                self.get_logger().error('A/B 点间距不足 5cm，请重新采 B 点')
        elif msg.data[0] == 3.0:
            self.update_zone_data("木桥A",self.x_trans,self.y_trans)
        elif msg.data[0] == 4.0:
            self.update_zone_data("木桥B",self.x_trans,self.y_trans)
        elif msg.data[0] == 5.0:
            self.update_zone_data("大斜坡",self.x_trans,self.y_trans)
        elif msg.data[0] == 6.0:
            self.update_zone_data("限高栏",self.x_trans,self.y_trans)
        elif msg.data[0] == 7.0:
            self.update_zone_data("沙坑",self.x_trans,self.y_trans)
        elif msg.data[0] == 8.0:
            self.update_zone_data("绕杆",self.x_trans,self.y_trans)
        elif msg.data[0] == 9.0:
            self.update_zone_data("高墙",self.x_trans,self.y_trans)
        elif msg.data[0] == 10.0:
            self.update_zone_data("台阶",self.x_trans,self.y_trans)
    def trans_pos(self, trans, pos_msg):
        trans.header.frame_id = "lidar_init"
        trans.header.stamp = pos_msg.header.stamp
        trans.child_frame_id = "body"
        
        trans.transform.translation.x = pos_msg.pose.pose.position.x
        trans.transform.translation.y = pos_msg.pose.pose.position.y
        trans.transform.translation.z = pos_msg.pose.pose.position.z
        
        trans.transform.rotation.x = pos_msg.pose.pose.orientation.x
        trans.transform.rotation.y = pos_msg.pose.pose.orientation.y
        trans.transform.rotation.z = pos_msg.pose.pose.orientation.z
        trans.transform.rotation.w = pos_msg.pose.pose.orientation.w

    def Odometry_callback(self, pos_msg):
        position = pos_msg.pose.pose.position
        orientation = pos_msg.pose.pose.orientation
        values = (position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w)
        if not all(math.isfinite(value) for value in values):
            return
        if sum(value * value for value in values[3:]) < 1e-12:
            return
        self.last_odometry_time = time.monotonic()
        trans = TransformStamped()
        self.trans_pos(trans, pos_msg)
        trans_p = trans.transform.translation
        trans_o = trans.transform.rotation

        self.x_p = trans_p.x
        self.y_p = trans_p.y
        self.z_p = trans_p.z

        x_o = trans_o.x
        y_o = trans_o.y
        z_o = trans_o.z
        w_o = trans_o.w

        norm_squared = x_o * x_o + y_o * y_o + z_o * z_o + w_o * w_o
        yaw = math.atan2(2.0 * (w_o * z_o + x_o * y_o),
                         norm_squared - 2.0 * (y_o * y_o + z_o * z_o))
        ready = self.first_captured and self.second_captured
        self.calibration_publisher.publish(Bool(data=ready))
        if not ready:
            return
        #yaw = math.degrees(yaw)

        if self.x1==self.x2 and self.y1==self.y2:
            self.x_trans, self.y_trans = self.x_p, self.y_p
            yaw_offset = 0
        else:    
            L = math.sqrt((self.x2-self.x1)**2 + (self.y2-self.y1)**2)
            self.x_trans, self.y_trans,yaw_offset = self.Param_Trans(self.x1, self.y1, self.x2, self.y2, self.x_p, self.y_p, L)
        # self.get_logger().info(f"{x_trans},{y_trans},yaw:{yaw},yaw_offset:{yaw_offset}\n")
        yaw = yaw - yaw_offset
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
        yaw = math.degrees(yaw)
        msg = Float32MultiArray()
        msg.data = [float(self.x_trans), float(self.y_trans), float(yaw)]
        self.publisher.publish(msg)
        #self.x_p = x_trans
        #self.y_p = y_trans
    def Param_Trans(self, x1, y1, x2, y2, x_p, y_p, L):

        x_trans = ((x_p - x1)*(x2 - x1) + (y_p - y1)*(y2 - y1))/L
        y_trans = ((x_p - x1)*(y2 - y1) - (y_p - y1)*(x2 - x1))/L
        yaw_offset = math.atan2((y2 - y1), (x2 - x1))

        return x_trans,y_trans,yaw_offset
        
        

def main(args=None):
    rclpy.init(args=args)
    node = TopicRenamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
