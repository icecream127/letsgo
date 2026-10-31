"""Opt-in ROS/PCL smoke tests. Run in an isolated ROS_DOMAIN_ID; no hardware."""
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time
from types import SimpleNamespace

import numpy as np
import pytest

from test_maps import maps, write_pcd


pytestmark = pytest.mark.skipif(os.environ.get('LETSGO_MAP_RUNTIME_TEST') != '1',
                                reason='Set LETSGO_MAP_RUNTIME_TEST=1 in an isolated ROS domain')


def test_save_client_parameters_and_trigger(tmp_path):
    """Exercise the real save() client against Humble parameter and Trigger services."""
    import rclpy
    from std_srvs.srv import Trigger
    rclpy.init()
    server = rclpy.create_node('laser_mapping')
    destination = tmp_path / 'maps' / 'client_test' / 'cloud.pcd'
    server.declare_parameter('map_file_path', str(destination))
    server.declare_parameter('pcd_save.pcd_save_en', True)
    calls = []

    def save_cloud(request, response):
        calls.append(True)
        write_pcd(destination, [[0, 0, 0], [1, 1, .5]], binary=True)
        response.success = True
        return response

    server.create_service(Trigger, '/map_save', save_cloud)
    code = ('import sys; from pathlib import Path; from types import SimpleNamespace; '
            'sys.path.insert(0, sys.argv[1]); import maps; maps.PROJECT=Path(sys.argv[2]); '
            'maps.save(SimpleNamespace(name="client_test", force=False))')
    child = subprocess.Popen(['/usr/bin/python3', '-c', code,
                              str(Path(__file__).parents[1] / 'run'), str(tmp_path)],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + 20.
        while child.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(server, timeout_sec=.1)
        if child.poll() is None:
            child.terminate()
        output = child.communicate(timeout=5)[0]
        assert child.returncode == 0, output
        assert len(calls) == 1
        assert '有效点数 2' in output
        assert destination.exists()
    finally:
        if child.poll() is None:
            child.kill()
            child.wait()
        server.destroy_node()
        rclpy.shutdown()


def test_actual_pcl_save_and_nav2_load(tmp_path, monkeypatch):
    """Compile the actual save function, project its PCL binary output, load in Nav2."""
    project = Path(__file__).parents[1]
    source = (project / 'lidar_ws/src/fastlio/src/laserMapping.cpp').read_text()
    save_function = source.split('void save_to_pcd()', 1)[1].split('template<typename T>', 1)[0]
    code = '''
#include <filesystem>
#include <stdexcept>
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
std::string map_file_path;
pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_wait_pub(new pcl::PointCloud<pcl::PointXYZI>);
void save_to_pcd()''' + save_function + '''
int main(int argc, char** argv) {
    map_file_path = argv[1];
    bool empty_failed = false;
    try { save_to_pcd(); } catch (const std::exception&) { empty_failed = true; }
    if (!empty_failed) return 10;
    for (auto xyz : {std::array<float,3>{0,0,0}, {0,0,.5}, {1,1,0}}) {
        pcl::PointXYZI p; p.x=xyz[0]; p.y=xyz[1]; p.z=xyz[2]; p.intensity=1;
        pcl_wait_pub->push_back(p);
    }
    save_to_pcd();
    map_file_path = std::string(argv[1]) + "/invalid.pcd";
    try { save_to_pcd(); } catch (const std::exception&) { return 0; }
    return 11;
}
'''
    cpp = tmp_path / 'save.cpp'
    cpp.write_text('#include <array>\n' + code)
    flags = subprocess.check_output(['pkg-config', '--cflags', '--libs', 'pcl_io-1.12'], text=True)
    executable = tmp_path / 'check_save'
    subprocess.run(['g++', '-std=c++17', str(cpp), '-o', str(executable), *shlex.split(flags)], check=True)
    monkeypatch.setattr(maps, 'PROJECT', tmp_path)
    destination = maps.map_directory('synthetic') / 'cloud.pcd'
    subprocess.run([str(executable), str(destination)], check=True)
    np.testing.assert_allclose(maps.read_pcd(destination), [[0, 0, 0], [0, 0, .5], [1, 1, 0]])
    assert not destination.with_name('cloud.pcd.tmp').exists()
    maps.project(SimpleNamespace(name='synthetic', force=False, resolution=1., ground_z=0.,
                                 ground_tolerance=.05, min_height=.1, max_height=2., padding=0.))
    yaml_path = maps.validate('synthetic')

    import rclpy
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    log = (tmp_path / 'nav2.log').open('w')
    launch = project / 'lidar_ws/src/ros2_to_stm32/launch/nav2_map.launch.py'
    process = subprocess.Popen(['ros2', 'launch', str(launch), f'map:={yaml_path}'],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('map_smoke_probe')
    try:
        state = node.create_client(GetState, '/map_server/get_state')
        assert state.wait_for_service(timeout_sec=20.)
        deadline = time.monotonic() + 20.
        active = False
        observed_states = []
        while time.monotonic() < deadline:
            future = state.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.)
            observed_states.append(str(future.result()) if future.done() else 'timeout')
            if future.done() and future.result().current_state.id == 3:
                active = True
                break
            rclpy.spin_once(node, timeout_sec=.1)
        assert active, f'map_server did not activate: {observed_states}'
        received = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(OccupancyGrid, '/map', received.append, qos)
        deadline = time.monotonic() + 10.
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.2)
        assert received, 'late subscriber did not receive latched map'
        msg = received[0]
        assert msg.header.frame_id == 'map'
        assert (msg.info.width, msg.info.height, msg.info.resolution) == (2, 2, 1.)
        assert (msg.info.origin.position.x, msg.info.origin.position.y) == (0., 0.)
        assert list(msg.data) == [100, -1, -1, 0]
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        log.close()


def test_live_fastlio_rejects_empty_save(tmp_path):
    """Run only the built estimator (no sensor driver), verify its real service."""
    import rclpy
    from std_srvs.srv import Trigger
    project = Path(__file__).parents[1]
    config = project / 'lidar_ws/src/ros2_to_stm32/config/fast_lio_mid360_real.yaml'
    output = tmp_path / 'empty.pcd'
    log = (tmp_path / 'fastlio.log').open('w')
    process = subprocess.Popen([
        'ros2', 'run', 'fast_lio', 'fastlio_mapping', '--ros-args', '--params-file', str(config),
        '-p', 'pcd_save.pcd_save_en:=true', '-p', f'map_file_path:={output}'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('empty_save_probe')
    try:
        client = node.create_client(Trigger, '/map_save')
        assert client.wait_for_service(timeout_sec=15.)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.)
        assert future.done(), 'save service response timed out'
        response = future.result()
        assert not response.success
        assert 'No accumulated map points' in response.message
        assert not output.exists()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        log.close()
