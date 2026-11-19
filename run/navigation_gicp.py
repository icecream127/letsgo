"""Start Nav2 with a saved base pose converted to a GICP IMU seed."""
import argparse
import json
import math
import os
from pathlib import Path

import yaml

from initialize_start import ROOT, load_start
from maps import validate
from task_files import PROJECT_NAME
from robot_params import ROBOT_PARAMS, script_params


def imu_seed(pose, offset, yaw_offset, z, roll, pitch):
    if len(offset) != 3 or not all(math.isfinite(v) for v in (*offset, yaw_offset, z, roll, pitch)):
        raise ValueError('无效的安装参数或 IMU 初始姿态')
    # Heading comes from the horizontal projection of the tilted IMU's
    # base-forward axis, matching the yaw extraction in nav_adapter.
    forward_x = math.cos(pitch)*math.cos(yaw_offset) + math.sin(pitch)*math.sin(roll)*math.sin(yaw_offset)
    forward_y = math.cos(roll)*math.sin(yaw_offset)
    if math.hypot(forward_x, forward_y) < 1e-9:
        raise ValueError('底盘前向接近竖直，无法推算平面启动朝向')
    heading = pose['yaw'] - math.atan2(forward_y, forward_x)
    heading = math.atan2(math.sin(heading), math.cos(heading))
    x, y, height = offset
    y, height = math.cos(roll)*y - math.sin(roll)*height, math.sin(roll)*y + math.cos(roll)*height
    x = math.cos(pitch)*x + math.sin(pitch)*height
    dx = math.cos(heading)*x - math.sin(heading)*y
    dy = math.sin(heading)*x + math.cos(heading)*y
    return [pose['x']-dx, pose['y']-dy, z, roll, pitch, heading]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_name', nargs='?', default='competition')
    parser.add_argument('--project', type=Path, help=f'默认使用地图目录下的 {PROJECT_NAME}')
    parser.add_argument('--imu-z', required=True, type=float, help='雷达/IMU 在 PCD 的 map 坐标中的初始 z，单位米')
    parser.add_argument('--imu-roll', type=float, default=0., help='初始 IMU 横滚角，弧度')
    parser.add_argument('--imu-pitch', type=float, default=0., help='初始 IMU 俯仰角，弧度')
    parser.add_argument('--params-file', type=Path, default=ROBOT_PARAMS)
    parser.add_argument('--check-only', action='store_true')
    parser.add_argument('--no-sensors', action='store_true')
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--use-sim-time', action='store_true')
    args = parser.parse_args()
    try:
        map_yaml = validate(args.map_name).resolve()
        pose = load_start(args.project or map_yaml.with_name(PROJECT_NAME), map_yaml)
        projection = json.loads(map_yaml.with_name('projection.json').read_text())
        pcd = (map_yaml.parent / projection['source']).resolve()
        params = args.params_file.resolve()
        mounting = script_params('nav_adapter')
        if params != ROBOT_PARAMS.resolve():
            profile = yaml.safe_load(params.read_text(encoding='utf-8')) or {}
            mounting = {**mounting, **profile.get('nav_adapter', {}).get('ros__parameters', {})}
        seed = imu_seed(pose, mounting['livox_to_base_xyz'], mounting['livox_to_base_yaw'],
                        args.imu_z, args.imu_roll, args.imu_pitch)
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    print(json.dumps({'pcd': str(pcd), 'base_pose': pose, 'imu_seed': seed}, ensure_ascii=False), flush=True)
    if args.check_only:
        return
    command = ['ros2', 'launch', 'ros2_to_stm32', 'navigation.launch.py',
               'localization:=gicp', f'map:={map_yaml}', f'pcd:={pcd}',
               f'params_file:={params}', f'gicp_seed:={json.dumps(seed)}',
               f'sensors:={str(not args.no_sensors).lower()}', f'rviz:={str(not args.no_rviz).lower()}',
               f'use_sim_time:={str(args.use_sim_time).lower()}']
    os.execvp(command[0], command)


if __name__ == '__main__':
    main()
