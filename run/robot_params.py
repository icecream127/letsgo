"""Read the real robot profile from scripts that do not start a ROS node."""
from pathlib import Path

import yaml


ROBOT_PARAMS = Path(__file__).resolve().parents[1] / 'lidar_ws/src/ros2_to_stm32/config/robot.yaml'


def script_params(section, params_file=ROBOT_PARAMS):
    profile = yaml.safe_load(ROBOT_PARAMS.read_text(encoding='utf-8'))
    params = profile[section]['ros__parameters']
    if params_file.resolve() == ROBOT_PARAMS.resolve():
        return params
    override = yaml.safe_load(params_file.read_text(encoding='utf-8')) or {}
    return {**params, **override.get(section, {}).get('ros__parameters', {})}
