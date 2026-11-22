"""Compute an RViz preview with the Nav2 planner, without commanding motion."""
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile

from nav_goals import goal_pose


class RoutePreview:
    def __init__(self, node, goals):
        self.node = node
        self.goals = goals
        self.client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = node.create_publisher(Path, '/auto_nav/preview_path', qos)

    def wait(self, future):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.)
        if not future.done():
            raise TimeoutError('规划请求超时')
        return future.result()

    def publish(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.node.get_clock().now().to_msg()
        for index, target in enumerate(self.goals):
            request = ComputePathToPose.Goal()
            request.goal = goal_pose(self.node, target)
            if index > 0:
                request.use_start = True
                request.start = goal_pose(self.node, self.goals[index - 1])
            try:
                handle = self.wait(self.client.send_goal_async(request))
                if not handle.accepted:
                    raise RuntimeError('规划器拒绝请求')
                result = self.wait(handle.get_result_async())
                if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.path.poses:
                    raise RuntimeError(f'规划失败，Nav2 状态 {result.status}')
            except (TimeoutError, RuntimeError) as exc:
                print(f'路线预览第 {index + 1}/{len(self.goals)} 段：{exc}', flush=True)
                break
            path.poses.extend(result.result.path.poses)
        else:
            print(f'路线预览已生成（{len(self.goals)} 段）。', flush=True)
        if path.poses:
            self.publisher.publish(path)

    def destroy(self):
        self.client.destroy()
