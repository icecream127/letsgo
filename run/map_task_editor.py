#!/usr/bin/env python3
"""Build a self-contained offline task editor. Does not start ROS or change maps."""
import argparse
import base64
import hashlib
import io
import json
import shutil
import subprocess
import webbrowser
from pathlib import Path
import yaml
from PIL import Image

from task_editor_server import serve_editor

ROOT = Path(__file__).resolve().parents[1]


def render_editor(map_yaml, server_mode=False):
    map_yaml = Path(map_yaml).resolve()
    info = yaml.safe_load(map_yaml.read_text())
    if info.get('mode', 'trinary') != 'trinary':
        raise ValueError('目前仅支持 trinary 地图')
    image_path = map_yaml.parent / info['image']
    img = Image.open(image_path)
    if img.mode != 'L':
        raise ValueError('需要 8 位灰度地图')
    import math
    resolution = float(info['resolution'])
    origin = info['origin']
    if resolution <= 0 or not math.isfinite(resolution) or len(origin) != 3 or not all(math.isfinite(v) for v in origin):
        raise ValueError('地图分辨率或 origin 无效')
    free, occupied = float(info['free_thresh']), float(info['occupied_thresh'])
    if not 0 <= free < occupied <= 1:
        raise ValueError('地图阈值无效')
    buffer = io.BytesIO()
    img.save(buffer, format='PNG')
    meta = dict(server_mode=server_mode, name=map_yaml.parent.name, width=img.width, height=img.height,
                resolution=resolution, origin=origin, negate=bool(info.get('negate', 0)),
                free_thresh=free, occupied_thresh=occupied,
                fingerprint=hashlib.sha256(map_yaml.read_bytes()+image_path.read_bytes()).hexdigest(),
                pixels=base64.b64encode(img.tobytes()).decode(),
                image='data:image/png;base64,'+base64.b64encode(buffer.getvalue()).decode())
    template = (ROOT/'tools/map_task_editor/editor.html').read_text(encoding='utf-8-sig')
    engine = (ROOT/'tools/map_task_editor/geometry.js').read_text(encoding='utf-8-sig')
    html = template.replace('/*__ENGINE__*/', engine).replace('/*__MAP__*/', 'const MAP='+json.dumps(meta).replace('<', '\\u003c')+';')
    return html, meta['fingerprint']


def build(map_yaml, output):
    html, _ = render_editor(map_yaml)
    output = Path(output).resolve()
    if output.suffix.lower() != '.html':
        raise ValueError('输出文件必须为 .html')
    if output.is_relative_to(ROOT/'maps'):
        raise ValueError('请将编辑器输出到 maps 目录之外')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html, encoding='utf-8')
    return output


def open_editor(target):
    if shutil.which('wslpath') and shutil.which('powershell.exe'):
        if isinstance(target, Path):
            target = subprocess.check_output(
                ['wslpath', '-w', str(target)], encoding='utf-8'
            ).rstrip('\r\n')
        literal_target = target.replace("'", "''")
        command = (
            "$ErrorActionPreference = 'Stop'; $ProgressPreference = 'SilentlyContinue'; "
            f"Start-Process -FilePath '{literal_target}'"
        )
        encoded_command = base64.b64encode(command.encode('utf-16-le')).decode('ascii')
        subprocess.run(
            ['powershell.exe', '-NoProfile', '-NonInteractive', '-EncodedCommand', encoded_command],
            check=True,
        )
    elif not webbrowser.open(target.as_uri() if isinstance(target, Path) else target):
        raise RuntimeError('无法启动默认浏览器')


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map', default=str(ROOT/'maps/competition/map.yaml'))
    p.add_argument('--output', default=str(ROOT/'tools/map_task_editor/competition_editor.html'))
    p.add_argument('--open', action='store_true', help='生成后打开采点界面')
    p.add_argument('--serve', action='store_true', help='启动本地采点服务，直接保存到地图目录')
    a=p.parse_args()
    if a.serve:
        html, fingerprint = render_editor(a.map, server_mode=True)
        serve_editor(a.map, html, fingerprint, open_editor if a.open else None)
        raise SystemExit(0)
    output = build(a.map, a.output)
    print(output, flush=True)
    if a.open:
        try:
            open_editor(output)
        except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
            p.exit(1, f'采点界面已生成，但未能自动打开：{exc}。可在浏览器打开上面的路径。\n')
