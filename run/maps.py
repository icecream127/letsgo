#!/usr/bin/env python3
"""Project-local PCD storage, conservative occupancy projection and map validation."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys

import numpy as np
import yaml

PROJECT = Path(__file__).resolve().parents[1]


def map_directory(name):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', name):
        raise ValueError('地图名仅允许字母、数字、下划线、短横线，长度 1..64')
    root = (PROJECT / 'maps').resolve()
    target = (root / name).resolve()
    if target.parent != root:
        raise ValueError('地图目录不能通过符号链接指向 maps 之外')
    return target


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read_pcd(path):
    """Read PCL writeBinary and ASCII PCD, including padding/extra COUNT fields."""
    with Path(path).open('rb') as stream:
        header = {}
        for _ in range(100):
            line = stream.readline().decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            key, *values = line.split()
            header[key.upper()] = values
            if key.upper() == 'DATA':
                break
        else:
            raise ValueError('PCD header missing DATA')
        fields = header['FIELDS']
        sizes = list(map(int, header['SIZE']))
        counts = list(map(int, header.get('COUNT', ['1'] * len(fields))))
        types = header['TYPE']
        if not len(fields) == len(sizes) == len(types) == len(counts):
            raise ValueError('PCD field declarations disagree')
        if any(c <= 0 for c in counts):
            raise ValueError('Invalid PCD COUNT')
        points = int(header.get('POINTS', [str(int(header['WIDTH'][0]) * int(header.get('HEIGHT', ['1'])[0]))])[0])
        if points <= 0:
            raise ValueError('PCD contains no points')
        for axis in ('x', 'y', 'z'):
            if axis not in fields or counts[fields.index(axis)] != 1:
                raise ValueError('PCD requires scalar x/y/z fields')
        data_type = header['DATA'][0].lower()
        if data_type == 'ascii':
            data = np.loadtxt(stream, ndmin=2)
            if data.shape != (points, sum(counts)):
                raise ValueError('PCD ASCII body size disagrees with header')
            offsets = np.cumsum([0] + counts[:-1])
            xyz = data[:, [offsets[fields.index(axis)] for axis in ('x', 'y', 'z')]]
        elif data_type == 'binary':
            kinds = {'F': 'f', 'I': 'i', 'U': 'u'}
            dtype = np.dtype([(f'f{i}', '<' + kinds[t] + str(s), (c,))
                              for i, (s, t, c) in enumerate(zip(sizes, types, counts))])
            body = stream.read()
            if len(body) != points * dtype.itemsize:
                raise ValueError('PCD binary body size disagrees with header')
            data = np.frombuffer(body, dtype=dtype, count=points)
            xyz = np.column_stack([data[f'f{fields.index(axis)}'][:, 0] for axis in ('x', 'y', 'z')])
        else:
            raise ValueError('仅支持 ASCII/binary PCD；binary_compressed 请先用 PCL 转换为 binary')
    xyz = np.asarray(xyz, dtype=np.float64)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if not len(xyz):
        raise ValueError('PCD has no finite XYZ points')
    return xyz


def project_cloud(points, resolution, ground_z, ground_tolerance, min_height, max_height, padding):
    values = (resolution, ground_z, ground_tolerance, min_height, max_height, padding)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Projection parameters must be finite')
    if resolution <= 0 or ground_tolerance <= 0 or padding < 0 or not ground_tolerance < min_height < max_height:
        raise ValueError('Require resolution>0, padding>=0 and 0<ground_tolerance<min_height<max_height')
    height = points[:, 2] - ground_z
    ground = np.abs(height) <= ground_tolerance
    obstacle = (height >= min_height) & (height <= max_height)
    if not ground.any():
        raise ValueError('没有地面点：请检查 --ground-z；不会把包围盒内空白区域全部设为空闲')
    usable = points[ground | obstacle]
    origin = np.floor((usable[:, :2].min(axis=0) - padding) / resolution) * resolution
    dimensions = np.floor((usable[:, :2].max(axis=0) + padding - origin) / resolution).astype(int) + 1
    width, height = map(int, dimensions)
    if width * height > 16_000_000:
        raise ValueError('Grid exceeds 16 million cells; crop outliers or increase resolution')
    grid = np.full((height, width), 205, dtype=np.uint8)  # unknown, not presumed free
    for selected, value in ((ground, 254), (obstacle, 0)):
        xy = np.floor((points[selected, :2] - origin) / resolution).astype(int)
        grid[xy[:, 1], xy[:, 0]] = value  # occupied takes precedence over ground
    # PGM rows run top->bottom; OccupancyGrid origin is the bottom-left corner.
    return np.flipud(grid), [float(origin[0]), float(origin[1]), 0.0]


def project(args):
    directory = map_directory(args.name)
    source = directory / 'cloud.pcd'
    outputs = [directory / name for name in ('map.pgm', 'map.yaml', 'projection.json')]
    if not args.force and any(p.exists() for p in outputs):
        raise ValueError('二维地图已存在；如需重新生成，显式添加 --force')
    source_hash = sha256(source)
    points = read_pcd(source)
    image, origin = project_cloud(points, args.resolution, args.ground_z,
                                 args.ground_tolerance, args.min_height, args.max_height, args.padding)
    if sha256(source) != source_hash:
        raise ValueError('PCD changed during projection; stop saving and retry')
    height, width = image.shape
    pgm = f'P5\n{width} {height}\n255\n'.encode('ascii') + image.tobytes()
    map_config = dict(image='map.pgm', mode='trinary', resolution=args.resolution,
                      origin=origin, negate=0, occupied_thresh=0.65, free_thresh=0.196)
    yaml_bytes = yaml.safe_dump(map_config, sort_keys=False).encode('utf-8')
    metadata = dict(created_utc=datetime.now(timezone.utc).isoformat(), source='cloud.pcd',
                    source_sha256=source_hash, image_sha256=hashlib.sha256(pgm).hexdigest(),
                    yaml_sha256=hashlib.sha256(yaml_bytes).hexdigest(),
                    coordinate_frame='saved FAST-LIO odom axes, served as map; localization must align live odom',
                    ground_z=args.ground_z, ground_tolerance=args.ground_tolerance,
                    min_height=args.min_height, max_height=args.max_height,
                    resolution=args.resolution, origin=origin, width=width, height=height,
                    cells=dict(free=int((image == 254).sum()), occupied=int((image == 0).sum()),
                               unknown=int((image == 205).sum())),
                    limitations='Flat-ground projection; no ray tracing, loop closure or traversability inference.')
    payloads = [pgm, yaml_bytes, (json.dumps(metadata, ensure_ascii=False, indent=2) + '\n').encode('utf-8')]
    temporary = [p.with_name(p.name + '.tmp') for p in outputs]
    try:
        for path, content in zip(temporary, payloads):
            path.write_bytes(content)
        for path, final in zip(temporary, outputs):
            path.replace(final)
    finally:
        for path in temporary:
            path.unlink(missing_ok=True)
    print(f'已生成 {directory / "map.yaml"}，{width}×{height}，格子统计：{metadata["cells"]}')


def validate(name):
    directory = map_directory(name)
    config_path = directory / 'map.yaml'
    config = yaml.safe_load(config_path.read_text())
    metadata = json.loads((directory / 'projection.json').read_text())
    if config['image'] != 'map.pgm':
        raise ValueError('地图 image 必须为同目录 map.pgm')
    for filename, field in [('cloud.pcd', 'source_sha256'), ('map.pgm', 'image_sha256'), ('map.yaml', 'yaml_sha256')]:
        if sha256(directory / filename) != metadata[field]:
            raise ValueError(f'{filename} 与生成记录不一致；请重新生成二维地图')
    return config_path


def save(args):
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from std_srvs.srv import Trigger
    directory = map_directory(args.name)
    destination = directory / 'cloud.pcd'
    if destination.exists() and not args.force:
        raise ValueError('cloud.pcd 已存在；换地图名或显式添加 --force。覆盖后须重新生成二维地图')
    rclpy.init()
    node = rclpy.create_node('letsgo_map_save_client')
    try:
        client = node.create_client(GetParameters, '/laser_mapping/get_parameters')
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('找不到 /laser_mapping；先运行 mapping.sh')
        request = GetParameters.Request()
        request.names = ['map_file_path', 'pcd_save.pcd_save_en']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        if not future.done() or future.result() is None:
            raise RuntimeError('读取 FAST-LIO 参数超时')
        values = future.result().values
        if len(values) != 2 or not values[1].bool_value or Path(values[0].string_value) != destination:
            raise ValueError(f'当前 FAST-LIO 未启用此地图的保存；请使用 mapping.sh {args.name} 启动')
        directory.mkdir(parents=True, exist_ok=True)
        service = node.create_client(Trigger, '/map_save')
        if not service.wait_for_service(timeout_sec=5.):
            raise RuntimeError('/map_save 服务不可用')
        future = service.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=120.)
        if not future.done() or future.result() is None:
            raise RuntimeError('保存请求超时；服务端可能仍在写入，请先检查文件再重试')
        if not future.result().success:
            raise RuntimeError(future.result().message)
        points = read_pcd(destination)
        print(f'已验证保存：{destination}，有效点数 {len(points)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    for command in ('prepare', 'save', 'project', 'validate'):
        sub = commands.add_parser(command)
        sub.add_argument('name', help='maps 下的场地名')
        if command in ('save', 'project'):
            sub.add_argument('--force', action='store_true')
        if command == 'project':
            sub.add_argument('--ground-z', type=float, required=True, help='地面在保存点云坐标中的 Z，单位 m')
            sub.add_argument('--ground-tolerance', type=float, default=0.05)
            sub.add_argument('--min-height', type=float, default=0.10)
            sub.add_argument('--max-height', type=float, default=2.0)
            sub.add_argument('--resolution', type=float, default=0.05)
            sub.add_argument('--padding', type=float, default=0.5)
    args = parser.parse_args()
    try:
        if args.command == 'prepare':
            directory = map_directory(args.name)
            if (directory / 'cloud.pcd').exists():
                raise ValueError('此场地已有 cloud.pcd，请使用新地图名，防止覆盖')
            directory.mkdir(parents=True, exist_ok=True)
            print(directory)
        elif args.command == 'save':
            save(args)
        elif args.command == 'project':
            project(args)
        else:
            print(validate(args.name))
    except (ValueError, KeyError, OSError, RuntimeError) as error:
        parser.exit(1, f'地图操作失败：{error}\n')


if __name__ == '__main__':
    main()
