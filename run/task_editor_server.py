"""Serve the task editor on loopback and save its result beside the map."""
from functools import partial
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
from subprocess import CalledProcessError
from tempfile import NamedTemporaryFile
from threading import Lock

import yaml

from task_files import GOALS_NAME, PROJECT_NAME, map_fingerprint


def valid_pose(pose):
    return isinstance(pose, dict) and all(
        type(pose.get(key)) in (int, float) and math.isfinite(pose[key])
        for key in ('x', 'y', 'yaw')
    )


def atomic_write(path, data):
    with NamedTemporaryFile(dir=path.parent, delete=False) as stream:
        temp = Path(stream.name)
        try:
            stream.write(data)
            stream.close()
            os.replace(temp, path)
        finally:
            temp.unlink(missing_ok=True)


def json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode('utf-8')


def save_project(directory, fingerprint, payload):
    project = payload['project']
    if not isinstance(project, dict) or project.get('version') != 1 or project.get('frame_id') != 'map':
        raise ValueError('任务工程格式无效')
    if project.get('map_fingerprint') != fingerprint:
        raise ValueError('工程与当前地图不匹配，请重新打开采点界面')
    tasks = project.get('tasks')
    if not isinstance(tasks, list) or len(tasks) > 500:
        raise ValueError('任务列表无效')
    project_data = json_bytes(project)
    goals = []
    if payload.get('export_goals') is True:
        if not tasks:
            raise ValueError('没有可导出的目标')
        for task in tasks:
            if not isinstance(task, dict) or task.get('type') != 'nav' or not isinstance(task.get('name'), str):
                raise ValueError('只有普通导航任务可生成 Nav2 目标')
            for key, suffix in (('entry', '入口'), ('exit', '出口')):
                pose = task.get(key)
                if not valid_pose(pose):
                    raise ValueError(f'{task["name"]}缺少有效的{suffix}')
                goals.append(dict(name=f'{task["name"]}_{suffix}', x=pose['x'], y=pose['y'], yaw=pose['yaw']))
    if goals:
        goals_data = json_bytes(dict(
            frame_id='map', map_fingerprint=fingerprint, goals=goals,
            project_sha256=hashlib.sha256(project_data).hexdigest(),
        ))
        atomic_write(directory / GOALS_NAME, goals_data)
    atomic_write(directory / PROJECT_NAME, project_data)
    if not goals:
        (directory / GOALS_NAME).unlink(missing_ok=True)
    return bool(goals)


class EditorHandler(BaseHTTPRequestHandler):
    def __init__(self, *args, map_yaml, html, fingerprint, **kwargs):
        self.map_yaml = map_yaml
        self.html = html
        self.fingerprint = fingerprint
        super().__init__(*args, **kwargs)

    def reply(self, status, data, content_type='application/json; charset=utf-8'):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.headers.get('Host') != self.server.origin.removeprefix('http://'):
            return self.send_error(403)
        if self.path == '/':
            return self.reply(200, self.html, 'text/html; charset=utf-8')
        if self.path != '/project':
            return self.send_error(404)
        path = self.map_yaml.with_name(PROJECT_NAME)
        try:
            data = path.read_bytes()
        except FileNotFoundError:
            data = b'null'
        except OSError as exc:
            return self.reply(500, json_bytes({'error': str(exc)}))
        self.reply(200, data)

    def do_POST(self):
        if self.path != '/project':
            return self.send_error(404)
        if (self.headers.get('Host') != self.server.origin.removeprefix('http://')
                or self.headers.get('Origin') != self.server.origin):
            return self.send_error(403)
        if self.headers.get_content_type() != 'application/json':
            return self.send_error(415)
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 2_000_000:
                return self.send_error(413)
            payload = json.loads(self.rfile.read(length))
            with self.server.save_lock:
                current = map_fingerprint(self.map_yaml)
                if current != self.fingerprint:
                    raise ValueError('地图已更新，请重启采点命令')
                generated = save_project(self.map_yaml.parent, current, payload)
        except (TypeError, KeyError, ValueError, OverflowError, yaml.YAMLError) as exc:
            return self.reply(400, json_bytes({'error': str(exc)}))
        except OSError as exc:
            return self.reply(500, json_bytes({'error': str(exc)}))
        self.reply(200, json_bytes({'goals_generated': generated}))

    def log_message(self, format, *args):
        pass


def serve_editor(map_yaml, html, fingerprint, open_browser):
    handler = partial(EditorHandler, map_yaml=Path(map_yaml).resolve(),
                      html=html.encode('utf-8'), fingerprint=fingerprint)
    with ThreadingHTTPServer(('127.0.0.1', 0), handler) as server:
        server.save_lock = Lock()
        server.origin = f'http://127.0.0.1:{server.server_port}'
        print(f'采点界面：{server.origin}/  （Ctrl+C 关闭）', flush=True)
        if open_browser:
            try:
                open_browser(server.origin + '/')
            except (OSError, RuntimeError, CalledProcessError) as exc:
                print(f'可手动打开上面的地址：{exc}', flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
