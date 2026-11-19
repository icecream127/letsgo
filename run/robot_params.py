"""Read the real robot profile from scripts that do not start a ROS node."""
from pathlib import Path

import yaml


ROBOT_PARAMS = Path(__file__).resolve().parents[1] / 'lidar_ws/src/ros2_to_stm32/config/robot.yaml'


def script_params(section):
    profile = yaml.safe_load(ROBOT_PARAMS.read_text(encoding='utf-8'))
    return profile[section]['ros__parameters']
