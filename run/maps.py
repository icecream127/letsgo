#!/usr/bin/env python3
"""Project-local PCD storage, conservative occupancy projection and map validation."""
import argparse
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
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


def project_terrain(points, resolution, ground_z, ground_tolerance, min_height,
                    max_height, padding, terrain_resolution=.2, ground_quantile=.05,
                    max_step=.10, max_slope=15., seed_x=0., seed_y=0., seed_radius=.5):
    values=(resolution,ground_z,ground_tolerance,min_height,max_height,padding,terrain_resolution,ground_quantile,max_step,max_slope,seed_x,seed_y,seed_radius)
    if not all(math.isfinite(v) for v in values): raise ValueError('Terrain parameters must be finite')
    if not (0<resolution<=terrain_resolution and padding>=0 and 0<ground_tolerance<min_height<max_height and 0<=ground_quantile<=.25 and 0<=max_step<max_height and 0<max_slope<60 and seed_radius>0): raise ValueError('Invalid terrain parameters')
    points=np.asarray(points,dtype=float)
    points=points[np.isfinite(points).all(axis=1)]
    if not len(points):raise ValueError('PCD has no finite XYZ points')
    origin=np.floor((points[:,:2].min(axis=0)-padding)/resolution)*resolution
    width,height=np.floor((points[:,:2].max(axis=0)+padding-origin)/resolution).astype(int)+1
    if width*height>16000000:raise ValueError('Grid exceeds 16 million cells')
    fine=np.floor((points[:,:2]-origin)/resolution).astype(int)
    cells,inverse,counts=np.unique(fine,axis=0,return_inverse=True,return_counts=True)
    order=np.lexsort((points[:,2],inverse));starts=np.r_[0,np.cumsum(counts)[:-1]]
    elevation=np.full(len(cells),np.nan);support=np.zeros(len(cells),int)
    # Densest thin vertical band per OUTPUT cell, instead of the low tail of a
    # coarse cell that can straddle both sides of a raised walkway.
    band=ground_tolerance
    for i in range(len(cells)):
        zs=points[order[starts[i]:starts[i]+counts[i]],2]
        zs=zs[(zs>=ground_z-max(.2,ground_tolerance*3))&(zs<=ground_z+max_height)]
        if not len(zs):continue
        ends=np.searchsorted(zs,zs+band,side='right')
        j=int(np.argmax(ends-np.arange(len(zs))))
        sample=zs[j:ends[j]]
        elevation[i]=np.median(sample);support[i]=len(sample)
    centers=origin+(cells+.5)*resolution
    lookup={tuple(c):i for i,c in enumerate(cells)}
    supported=np.isfinite(elevation)
    slope=math.tan(math.radians(max_slope))
    gradients=np.zeros((len(cells),2));steep=np.zeros(len(cells),bool)
    # Fit a neighborhood surface only to elevations close to the center.
    # Height discontinuities are excluded so a step is not averaged into a ramp.
    radius=max(1,int(round(.15/resolution)))
    for i in np.flatnonzero(supported):
        x,y=cells[i];rows=[];zs=[]
        for dx in range(-radius,radius+1):
            for dy in range(-radius,radius+1):
                j=lookup.get((x+dx,y+dy))
                if j is not None and supported[j] and abs(elevation[j]-elevation[i])<=max_step+1e-8:
                    rows.append([dx*resolution,dy*resolution,1.]);zs.append(elevation[j])
        if len(rows)>=6:
            fit,_,rank,_=np.linalg.lstsq(rows,zs,rcond=None)
            if rank==3:
                rms=np.sqrt(np.mean((np.asarray(rows)@fit-zs)**2))
                if rms<=.012:
                    gradients[i]=fit[:2]
                    steep[i]=np.linalg.norm(fit[:2])>slope+1e-6
    supported &= ~steep
    seeds=np.flatnonzero(supported & (np.linalg.norm(centers-[seed_x,seed_y],axis=1)<=seed_radius) & (np.abs(elevation-ground_z)<=ground_tolerance*2))
    if not len(seeds):raise ValueError('种子附近没有地面点，或坡度超过限制')
    directions=((1,0),(-1,0),(0,1),(0,-1))
    reachable=np.zeros(len(cells),bool);reachable[seeds]=True;queue=deque(seeds.tolist())
    def connected(i,j):
        dz=elevation[j]-elevation[i]
        predicted=np.dot((gradients[i]+gradients[j])*.5,centers[j]-centers[i])
        return abs(dz)<=max(max_step,slope*resolution)+1e-7 and abs(dz-predicted)<=max_step+1e-7
    while queue:
        i=queue.popleft();x,y=cells[i]
        for dx,dy in directions:
            j=lookup.get((x+dx,y+dy))
            if j is not None and supported[j] and not reachable[j] and connected(i,j):
                reachable[j]=True;queue.append(j)
    known=reachable[inverse];relative=points[:,2]-elevation[inverse]
    ground=known & (np.abs(relative)<=ground_tolerance)
    obstacles=known & (relative>=max(min_height,max_step+ground_tolerance)) & (relative<=max_height)
    obstacles |= ~known & (points[:,2]>=ground_z-ground_tolerance) & (points[:,2]<=ground_z+max_height)
    grid=np.full((height,width),205,np.uint8)
    xy=fine[ground];grid[xy[:,1],xy[:,0]]=254
    # Retain measured vertical objects, including thin wall edges. Surface mode
    # selection already removes the coarse-cell false obstacle mechanism.
    xy=fine[obstacles]
    ids,number=np.unique(xy[:,1]*width+xy[:,0],return_counts=True)
    occupied=np.zeros((height,width),bool);occupied.ravel()[ids]=True
    neighbors=np.zeros_like(grid,dtype=int)
    for dy,dx in ((1,0),(-1,0),(0,1),(0,-1)):
        neighbors+=np.roll(occupied,(dy,dx),(0,1))
    strong=ids[(number>=3)|(neighbors.ravel()[ids]>=2)]
    grid.ravel()[strong]=0
    structural=fine[obstacles & ~known]
    grid[structural[:,1],structural[:,0]]=0
    cliffs=np.zeros(len(cells),bool)
    for i in np.flatnonzero(reachable):
        x,y=cells[i]
        for dx,dy in directions:
            j=lookup.get((x+dx,y+dy))
            if j is not None and np.isfinite(elevation[j]) and elevation[i]-elevation[j]>max_step+1e-7:
                # Higher side of an observed drop: one cell, never an entire
                # terrain voxel. The lower side remains available for routing.
                cliffs[i]=True;grid[y,x]=0;break
    info=dict(algorithm='fine_surface_mode_v2',terrain_resolution=resolution,ground_band=band,max_step=max_step,max_slope_degrees=max_slope,seed=[seed_x,seed_y,ground_z],seed_radius=seed_radius,surface_cells=int(reachable.sum()),cliff_cells=int(cliffs.sum()),steep_cells=int(steep.sum()),ground_elevation_range=[float(np.nanmin(elevation[reachable & (grid[cells[:,1],cells[:,0]] == 254)])),float(np.nanmax(elevation[reachable & (grid[cells[:,1],cells[:,0]] == 254)]))])
    return np.flipud(grid),[float(origin[0]),float(origin[1]),0.],info


def apply_confirmed_regions(image, origin, points, resolution, profile, max_height):
    """Project field-confirmed slatted terrain; never generalize gap capability.

    The surface profile is measured from the upper-board returns. Only the
    bounded region may bridge missing support. Objects above it retain priority.
    Both lateral edges are blocked while longitudinal entries remain open.
    """
    grid = np.flipud(image).copy()
    yy, xx = np.indices(grid.shape)
    X = origin[0] + (xx + .5) * resolution
    Y = origin[1] + (yy + .5) * resolution
    reports = []
    if profile.get('schema') != 1 or not profile.get('confirmation'):
        raise ValueError('Confirmed terrain profile requires schema=1 and confirmation')
    for region in profile['regions']:
        lo, hi = region['x_range']
        a, b, c = region['normal_equation']
        nlo, nhi = region['normal_range']
        zlo, zhi = region['surface_z_band']
        if not all(math.isfinite(v) for v in [lo, hi, a, b, c, nlo, nhi, zlo, zhi]) or not (lo < hi and nlo < nhi and zlo < zhi and b != 0):
            raise ValueError('Invalid confirmed region geometry')
        normal = a * X + b * Y + c
        inside = (X >= lo) & (X <= hi) & (normal >= nlo) & (normal <= nhi)
        pn = a * points[:, 0] + b * points[:, 1] + c
        selected = (points[:, 0] >= lo) & (points[:, 0] <= hi) & (pn >= nlo) & (pn <= nhi)
        observed = points[selected]
        top = observed[(observed[:, 2] >= zlo) & (observed[:, 2] <= zhi)]
        if len(top) < 20:
            raise ValueError('Confirmed region has insufficient board-surface observations')
        bins = np.arange(lo, hi + resolution, resolution)
        heights = np.full(len(bins) - 1, np.nan)
        idx = np.minimum(np.searchsorted(bins, top[:, 0], side='right') - 1, len(heights) - 1)
        for k in range(len(heights)):
            sample = top[idx == k, 2]
            if len(sample) >= 3:
                heights[k] = np.median(sample)
        centers = (bins[:-1] + bins[1:]) / 2
        valid = np.isfinite(heights)
        if valid.sum() < 3:
            raise ValueError('Insufficient longitudinal surface profile')
        surface = np.interp(observed[:, 0], centers[valid], heights[valid])
        obstacle = observed[(observed[:, 2] - surface >= .15) &
                            (observed[:, 2] - surface <= max_height)]
        before = grid.copy()
        grid[inside] = 254
        # Preserve observed walls and rails above the boards. A region-specific support count
        # prevents sparse residual returns from blocking the confirmed crossing.
        xy = np.floor((obstacle[:, :2] - origin[:2]) / resolution).astype(int)
        ids, counts = np.unique(xy[:, 1] * grid.shape[1] + xy[:, 0], return_counts=True)
        grid.ravel()[ids[counts >= region.get('min_obstacle_points', 8)]] = 0
        side = inside & ((normal - nlo < resolution * math.hypot(a, b)) |
                         (nhi - normal < resolution * math.hypot(a, b)))
        if 'front_open_x_range' in region:
            entry_lo, entry_hi = region['front_open_x_range']
            side &= ~((X >= entry_lo) & (X <= entry_hi) &
                      (nhi - normal < resolution * math.hypot(a, b)))
        if region.get('protect_sides', True):
            grid[side] = 0
        if 'protect_x_edges_above_normal' in region:
            lateral = inside & (normal > region['protect_x_edges_above_normal']) & (
                (X - lo < resolution) | (hi - X < resolution))
            grid[lateral] = 0
        reports.append(dict(name=region['name'], changed_cells=int((grid != before).sum()),
                            measured_profile_bins=int(valid.sum()), total_profile_bins=len(heights),
                            surface_z_range=[float(heights[valid].min()), float(heights[valid].max())],
                            entry=region.get('entry'), gap_width_limit=None,
                            interpretation='field-confirmed effective traversability; gaps are not physical floor'))
    return np.flipud(grid), reports


def project(args):
    directory = map_directory(args.name)
    source_name = getattr(args, 'source_map', None) or args.name
    source = map_directory(source_name) / 'cloud.pcd'
    source_relative = 'cloud.pcd' if source_name == args.name else f'../{source_name}/cloud.pcd'
    outputs = [directory / name for name in ('map.pgm', 'map.yaml', 'projection.json')]
    if not args.force and any(p.exists() for p in outputs):
        raise ValueError('二维地图已存在；如需重新生成，显式添加 --force')
    source_hash = sha256(source)
    points = read_pcd(source)
    bounds = getattr(args, 'bounds', None)
    if bounds is not None:
        xmin, ymin, xmax, ymax = bounds
        if not all(math.isfinite(v) for v in bounds) or not (xmin < xmax and ymin < ymax):
            raise ValueError('bounds require finite xmin < xmax, ymin < ymax')
        points = points[(points[:, 0] >= xmin) & (points[:, 0] < xmax) &
                        (points[:, 1] >= ymin) & (points[:, 1] < ymax)]
        if not len(points):
            raise ValueError('裁剪范围内没有点云')
    method = getattr(args, 'method', 'flat')
    terrain = {}
    if method == 'terrain':
        image, origin, terrain = project_terrain(
            points, args.resolution, args.ground_z, args.ground_tolerance,
            args.min_height, args.max_height, args.padding,
            args.terrain_resolution, args.ground_quantile, args.max_step,
            args.max_slope, args.seed_x, args.seed_y, args.seed_radius)
    else:
        image, origin = project_cloud(points, args.resolution, args.ground_z,
                                     args.ground_tolerance, args.min_height, args.max_height, args.padding)
    if bounds is not None:
        dimensions = (np.array(bounds[2:]) - bounds[:2]) / args.resolution
        offset = (np.array(bounds[:2]) - origin[:2]) / args.resolution
        if not (np.allclose(dimensions, np.rint(dimensions), atol=1e-7, rtol=0) and
                np.allclose(offset, np.rint(offset), atol=1e-7, rtol=0)):
            raise ValueError('bounds 必须与 resolution 栅格对齐')
        w, h = np.rint(dimensions).astype(int)
        if w * h > 16_000_000:
            raise ValueError('Grid exceeds 16 million cells')
        ix = np.arange(w) + int(round(offset[0]))
        iy = np.arange(h) + int(round(offset[1]))
        valid_x, valid_y = (ix >= 0) & (ix < image.shape[1]), (iy >= 0) & (iy < image.shape[0])
        cropped = np.full((h, w), 205, dtype=np.uint8)
        cropped[np.ix_(valid_y, valid_x)] = np.flipud(image)[np.ix_(iy[valid_y], ix[valid_x])]
        image, origin = np.flipud(cropped), [float(bounds[0]), float(bounds[1]), 0.]
    profile_path = directory / 'traversability.json'
    if profile_path.exists():
        profile = json.loads(profile_path.read_text())
        if profile.get('source_sha256') != source_hash:
            raise ValueError('已确认地形区域与源点云不匹配，需重新核实区域')
        image, reports = apply_confirmed_regions(image, origin, points, args.resolution, profile, args.max_height)
        terrain['confirmed_regions'] = reports
        terrain['profile_sha256'] = sha256(profile_path)
        terrain['profile_snapshot'] = profile
    if sha256(source) != source_hash:
        raise ValueError('PCD changed during projection; stop saving and retry')
    height, width = image.shape
    pgm = f'P5\n{width} {height}\n255\n'.encode('ascii') + image.tobytes()
    map_config = dict(image='map.pgm', mode='trinary', resolution=args.resolution,
                      origin=origin, negate=0, occupied_thresh=0.65, free_thresh=0.196)
    yaml_bytes = yaml.safe_dump(map_config, sort_keys=False).encode('utf-8')
    metadata = dict(created_utc=datetime.now(timezone.utc).isoformat(), source=source_relative,
                    source_map=source_name, bounds=bounds,
                    source_sha256=source_hash, image_sha256=hashlib.sha256(pgm).hexdigest(),
                    yaml_sha256=hashlib.sha256(yaml_bytes).hexdigest(),
                    coordinate_frame='saved FAST-LIO odom axes, served as map; localization must align live odom',
                    ground_z=args.ground_z, ground_tolerance=args.ground_tolerance,
                    min_height=args.min_height, max_height=args.max_height,
                    resolution=args.resolution, origin=origin, width=width, height=height,
                    cells=dict(free=int((image == 254).sum()), occupied=int((image == 0).sum()),
                               unknown=int((image == 205).sum())),
                    method=method, terrain=terrain,
                    limitations=('Single surface 2.5D offline estimate; no ray tracing or multi-level overlap support. '
                                 'Step/slope limits require chassis validation.' if method == 'terrain' else
                                 'Flat-ground projection; no ray tracing, loop closure or traversability inference.'))
    payloads = [pgm, yaml_bytes, (json.dumps(metadata, ensure_ascii=False, indent=2) + '\n').encode('utf-8')]
    temporary = [p.with_name(p.name + '.tmp') for p in outputs]
    if any(p.exists() for p in outputs):
        backup = directory / 'backups' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        backup.mkdir(parents=True)
        for path in outputs:
            if path.exists():
                shutil.copy2(path, backup / path.name)
        print(f'原二维地图已备份：{backup}')
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
    source_name = metadata.get('source_map', name)
    source = map_directory(source_name) / 'cloud.pcd'
    expected_source = 'cloud.pcd' if source_name == name else f'../{source_name}/cloud.pcd'
    if metadata.get('source') != expected_source:
        raise ValueError('源点云路径与 source_map 不一致')
    profile_hash = metadata.get('terrain', {}).get('profile_sha256')
    if profile_hash and sha256(directory / 'traversability.json') != profile_hash:
        raise ValueError('traversability.json 已改变，请重新生成二维地图')
    for path, field in [(source, 'source_sha256'), (directory / 'map.pgm', 'image_sha256'),
                        (directory / 'map.yaml', 'yaml_sha256')]:
        if sha256(path) != metadata[field]:
            filename = path.name
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
            sub.add_argument('--source-map', help='源点云所属 maps 地图名；输出仍写入 name')
            sub.add_argument('--bounds', type=float, nargs=4, metavar=('XMIN', 'YMIN', 'XMAX', 'YMAX'),
                             help='仅投影此 XY 范围内的点，保持原坐标')
            sub.add_argument('--method', choices=('terrain', 'flat'), default='terrain',
                             help='terrain 局部地形投影（默认）；flat 兼容旧固定高度切片')
            sub.add_argument('--terrain-resolution', type=float, default=0.20, help='兼容旧命令；地形单元现跟随 resolution')
            sub.add_argument('--ground-quantile', type=float, default=0.05, help='兼容旧命令；现使用密集高度带')
            sub.add_argument('--max-step', type=float, default=0.10, help='可跨单级高度 m，按用户底盘能力设定')
            sub.add_argument('--max-slope', type=float, default=15., help='最大坡度 deg，按用户底盘能力设定')
            sub.add_argument('--seed-x', type=float, default=0.)
            sub.add_argument('--seed-y', type=float, default=0.)
            sub.add_argument('--seed-radius', type=float, default=0.5)
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
