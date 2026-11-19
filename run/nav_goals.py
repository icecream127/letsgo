"""Send map-frame task goals through Nav2; independent of localization provider."""
import argparse
import hashlib
import json
import math
import time
from pathlib import Path


def load_goals(path, map_yaml=None):
    data = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    if map_yaml is not None:
        from task_files import PROJECT_NAME, map_fingerprint
        if data.get('map_fingerprint') != map_fingerprint(map_yaml):
            raise ValueError('目标与当前地图不匹配，请重新打开采点界面并保存')
        project = map_yaml.with_name(PROJECT_NAME).read_bytes()
        if data.get('project_sha256') != hashlib.sha256(project).hexdigest():
            raise ValueError('Nav2 目标与当前任务工程不一致，请检查普通导航任务并重新保存')
    if data.get('frame_id') != 'map':
        raise ValueError('frame_id 必须为 map')
    goals = data.get('goals')
    if not isinstance(goals, list) or not goals:
        raise ValueError('goals 必须为非空列表')
    for goal in goals:
        for key in ('x', 'y', 'yaw'):
            value = goal[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{key} 必须为有限数值，yaw 单位为弧度')
    return goals


def execute_sequence(goals, navigate):
    for index, goal in enumerate(goals):
        print(f'目标 {index + 1}/{len(goals)}: {goal}', flush=True)
        if not navigate(goal):
            print('任务停止：当前目标未成功到达，不发送后续目标。', flush=True)
            return False
    print('全部目标已到达。', flush=True)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    source.add_argument('--file', type=Path)
    source.add_argument('--map', help='读取指定地图保存的目标，默认 competition')
    parser.add_argument('--check-only', action='store_true')
    parser.add_argument('--timeout', type=float, help='覆盖实车参数文件中的单目标执行超时')
    args = parser.parse_args()
    if args.timeout is None:
        import yaml
        from robot_params import script_params

        try:
            args.timeout = float(script_params('nav_goal_client')['goal_timeout'])
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            parser.error(f'无法从 robot.yaml 读取 goal_timeout：{exc}')
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('timeout 必须为正数')
    try:
        if args.goal is not None:
            if not all(math.isfinite(v) for v in args.goal):
                raise ValueError('目标坐标必须为有限数值')
            goals = [dict(zip(('x', 'y', 'yaw'), args.goal))]
        elif args.file is not None:
            goals = load_goals(args.file)
        else:
            from maps import map_directory
            from task_files import GOALS_NAME, PROJECT_NAME
            directory = map_directory(args.map or 'competition')
            goals_path = directory / GOALS_NAME
            if not goals_path.is_file():
                if (directory / PROJECT_NAME).is_file():
                    raise ValueError(
                        f'{directory.name} 的任务工程已保存，但未生成 {GOALS_NAME}。'
                        '请在采点界面核对各任务均为普通导航、入口和出口齐全且通过占地检查，然后重新保存。'
                    )
                raise ValueError(f'{directory.name} 尚无任务工程。请先在采点界面标点并保存。')
            goals = load_goals(goals_path, directory / 'map.yaml')
    except (OSError, ValueError, TypeError, KeyError) as exc:
        parser.error(str(exc))
    if args.check_only:
        print(json.dumps({'frame_id': 'map', 'goals': goals}, ensure_ascii=False, indent=2))
        return 0

    import rclpy
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus
    rclpy.init()
    node = rclpy.create_node('task_goal_client')
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    active = None
    pending = None

    def wait(future, seconds):
        deadline = time.monotonic() + seconds
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
        if not future.done():
            raise TimeoutError('等待 Nav2 响应超时')
        return future.result()

    def navigate(goal):
        nonlocal active, pending
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = 'map'
        request.pose.header.stamp = node.get_clock().now().to_msg()
        p = request.pose.pose
        p.position.x, p.position.y = float(goal['x']), float(goal['y'])
        p.orientation.z = math.sin(goal['yaw'] / 2)
        p.orientation.w = math.cos(goal['yaw'] / 2)
        pending = client.send_goal_async(request)
        active = wait(pending, 10.)
        if not active.accepted:
            print('Nav2 拒绝目标。', flush=True)
            active = None
            return False
        result = wait(active.get_result_async(), args.timeout)
        active = None
        print(f'Nav2 结果状态：{result.status}', flush=True)
        return result.status == GoalStatus.STATUS_SUCCEEDED

    try:
        if not client.wait_for_server(timeout_sec=10.):
            raise TimeoutError('Nav2 navigate_to_pose 尚未启动')
        return 0 if execute_sequence(goals, navigate) else 1
    except (KeyboardInterrupt, TimeoutError) as exc:
        print(f'停止任务：{exc}', flush=True)
        return 1
    finally:
        # Resolve a pending acceptance before cancellation; never cancel other clients' goals.
        try:
            if active is None and pending is not None and not pending.done():
                active = wait(pending, 5.)
            if active is not None and active.accepted:
                reply = wait(active.cancel_goal_async(), 5.)
                if not reply.goals_canceling:
                    print('未确认取消，请检查 Nav2 状态。', flush=True)
        except Exception as exc:
            print(f'取消未确认：{exc}。请检查导航状态并按需停止底盘。', flush=True)
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
