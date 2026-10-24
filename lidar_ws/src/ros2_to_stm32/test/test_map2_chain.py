import json
from pathlib import Path
from unittest.mock import patch

import pytest
import rclpy
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger

from ros2_to_stm32.route_review import load_review, select_region
from ros2_to_stm32.zone_manager import zone_config_path
from ros2_to_stm32.task_to_do_unitree import Task_To_Do
from ros2_to_stm32.trans import TopicRenamer
from ros2_to_stm32.ui import TaskNode
from ros2_to_stm32.info_transmiting import Info_Trans
from test_motor_adaptation import bridge, arm, request


def test_map2_selected_and_entry_not_demo():
    data, fingerprint, issues = load_review(zone_config_path())
    assert Path(zone_config_path()).name == 'map_zones_config_2.json'
    assert data['zone']['区域1']['x_max'] == 1.85
    assert select_region(data, 0, 0)[0] == '木桥A'
    assert select_region(data, 1.9, -4)[0] == '木桥A'
    assert select_region(data, 100, 0) == (None, None)
    assert len(fingerprint) == 64
    assert any('木桥B' in item and '原点' in item for item in issues)


@pytest.fixture
def course():
    rclpy.init()
    node = Task_To_Do()
    node.motor_state = dict(armed=False, busy=False)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_default_course_requires_review(course):
    assert course.route_profile == 'zones'
    assert course.route_review_reason()
    with pytest.raises(RuntimeError, match='审核'):
        course.Quadruped_Obstacle(0, 0, 0)


def test_all_eight_regions_reachable_and_entry_dispatch(course):
    course.reviewed_map_sha256 = course.map_hash
    calls = []
    for name, region in course.zones.items():
        setattr(course, region['func_name'], lambda *args, name=name: calls.append(name))
    course.Quadruped_Obstacle(0, 0, 0)
    assert calls == ['木桥A']
    for name, region in course.zones.items():
        course.Quadruped_Obstacle((region['x_min'] + region['x_max']) / 2,
                                 (region['y_min'] + region['y_max']) / 2, 0)
        assert calls[-1] == name
    with pytest.raises(RuntimeError, match='离开'):
        course.Quadruped_Obstacle(100, 100, 0)


def test_map_change_reloads_and_invalidates_review(course, tmp_path):
    path = tmp_path / 'map.json'
    data = json.loads(Path(zone_config_path()).read_text())
    path.write_text(json.dumps(data))
    course.zm.filepath = str(path)
    course.refresh_route()
    course.reviewed_map_sha256 = course.map_hash
    data['zones']['木桥B']['begin_y'] = -4.13
    path.write_text(json.dumps(data))
    course.motor_state['armed'] = True
    with pytest.raises(RuntimeError, match='变化'):
        course.refresh_route()
    course.motor_state['armed'] = False
    course.refresh_route()
    assert course.zones['木桥B']['begin_y'] == -4.13
    assert course.route_review_reason()


def test_navigation_captures_dynamic_target_and_times_out(course):
    course.active_handler = 'handle_big_slope'
    course.Kkey_To_Number = lambda key: False
    with patch('ros2_to_stm32.task_to_do_unitree.time.monotonic', return_value=100):
        course.navigate_to_pose(0, 0, 0, 0, 1, 90)
    with patch('ros2_to_stm32.task_to_do_unitree.time.monotonic', return_value=101):
        course.navigate_to_pose(0.2, 0, 0, 0.2, 1, 90)
    assert course.navigation_state['target'] == (0, 1, 90)
    with patch('ros2_to_stm32.task_to_do_unitree.time.monotonic', return_value=200):
        with pytest.raises(RuntimeError, match='超时'):
            course.navigate_to_pose(0.2, 0, 0, 0.2, 1, 90)


def test_route_gate_blocks_arm_and_stale_route_pauses(bridge):
    node, port, clock = bridge
    node.route(String(data='{"ready":false,"reason":"unreviewed"}'))
    assert not node.enable(SetBool.Request(data=True), SetBool.Response()).success
    assert not port.sent
    node.route(String(data='{"ready":true}'))
    arm(node)
    node.route_time = clock.now - 2
    node.tick()
    assert not node.armed


def test_short_write_of_jump_keeps_guard(bridge):
    node, port, clock = bridge
    arm(node)
    port.short_write = True
    request(node, 6)
    node.tick()
    assert not node.armed
    assert node.busy_until > clock.now
    assert not node.enable(SetBool.Request(data=True), SetBool.Response()).success


def test_obstacle_capture_requires_ab_and_paused_state(tmp_path):
    rclpy.init()
    node = TopicRenamer()
    updates = []
    node.update_zone_data = lambda *args: updates.append(args)
    try:
        node.last_odometry_time = float('inf')
        node.task_param_callback(Float32MultiArray(data=[3.0]))
        assert not updates
        node.first_captured = node.second_captured = True
        node.x2 = 1.0
        node.x_p, node.y_p = 2.0, 1.0
        node.motor_status(String(data='{"armed":true,"busy":false}'))
        node.task_param_callback(Float32MultiArray(data=[3.0]))
        assert not updates
        node.motor_status(String(data='{"armed":false,"busy":false}'))
        node.task_param_callback(Float32MultiArray(data=[3.0]))
        assert updates == [('木桥A', 2.0, -1.0)]
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), 'bad'])
def test_invalid_map_coordinate_rejected(tmp_path, bad):
    data = json.loads(Path(zone_config_path()).read_text())
    data['zones']['木桥A']['begin_x'] = bad
    path = tmp_path / 'invalid.json'
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='坐标'):
        load_review(path)


def test_ui_uses_controller_region_priority_and_live_reset_service():
    rclpy.init()
    ui = TaskNode()
    controller = Info_Trans()
    try:
        assert ui.compute_local_coord(0, 0)[0] == '木桥A'
        assert ui.compute_local_coord(1.9, -4)[0] == '木桥A'
        controller.task_to_do.index = 4
        controller.task_to_do.motor_state = dict(armed=True, busy=False)
        result = controller.reset_route(Trigger.Request(), Trigger.Response())
        assert not result.success
        assert controller.task_to_do.index == 4
        controller.task_to_do.motor_state['armed'] = False
        result = controller.reset_route(Trigger.Request(), Trigger.Response())
        assert result.success
        assert controller.task_to_do.index == 0
        assert ui.reset_index_client.srv_name == '/automation/reset_route'
    finally:
        ui.destroy_node()
        controller.task_to_do.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


def test_special_action_preparation_is_nonblocking(course):
    course.route_profile = 'demo'
    course.motor_state['armed'] = True
    with patch('ros2_to_stm32.task_to_do_unitree.time.monotonic', return_value=100):
        course.begin_action(15, 6, course.bunker_exit_prepare_delay)
        course.Quadruped_Obstacle(0, 0, 0)
        assert course.command is None
        assert course.action_ready_at == 103
    with patch('ros2_to_stm32.task_to_do_unitree.time.monotonic', return_value=103):
        course.Quadruped_Obstacle(0, 0, 0)
        assert course.command['key'] == 6
