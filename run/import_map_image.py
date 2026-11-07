#!/usr/bin/env python3
"""Validate and import a GIMP-edited image without changing map coordinates."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import numpy as np
from PIL import Image
import maps


def import_image(name, filename, check_only=False):
    directory = maps.map_directory(name)
    maps.validate(name)
    source = Path(filename).resolve()
    if source == (directory / 'map.pgm').resolve():
        raise ValueError('请另存为 map_edited.pgm 后导入，不要直接覆盖正式 map.pgm')
    with Image.open(source) as edited, Image.open(directory / 'map.pgm') as original:
        if edited.mode != 'L' or edited.size != original.size:
            raise ValueError('要求 8 位灰度，且尺寸必须与正式地图相同')
        pixels = np.asarray(edited).copy()
    if not set(np.unique(pixels).tolist()) <= {0, 205, 254}:
        raise ValueError('只允许灰度 0、205、254；请检查抗锯齿、羽化和前景色')
    if check_only:
        return '尺寸、灰度和原地图校验通过；未修改文件'
    meta = json.loads((directory / 'projection.json').read_text())
    timestamp = datetime.now(timezone.utc)
    backup = maps.PROJECT / '.local/map_backups' / name / timestamp.strftime('%Y%m%dT%H%M%S%fZ')
    backup.mkdir(parents=True)
    for item in ('map.pgm', 'map.yaml', 'projection.json', 'traversability.json'):
        path = directory / item
        if path.exists():
            shutil.copy2(path, backup / item)
    previous_hash = meta['image_sha256']
    payload = source.read_bytes()
    import hashlib
    new_hash = hashlib.sha256(payload).hexdigest()
    meta['image_sha256'] = new_hash
    meta['method'] = 'manual_edit'
    meta['cells'] = dict(free=int((pixels == 254).sum()), occupied=int((pixels == 0).sum()), unknown=int((pixels == 205).sum()))
    meta['manual_edit'] = dict(imported_utc=timestamp.isoformat(), input=str(source), input_sha256=new_hash,
                               previous_image_sha256=previous_hash, backup=str(backup),
                               coordinate_geometry='unchanged size, resolution and origin')
    (directory / 'map.pgm.tmp').write_bytes(payload)
    (directory / 'projection.json.tmp').write_text(json.dumps(meta, ensure_ascii=False, indent=2) + '\n')
    (directory / 'map.pgm.tmp').replace(directory / 'map.pgm')
    (directory / 'projection.json.tmp').replace(directory / 'projection.json')
    reports = maps.PROJECT / 'validation/results' / name
    for report in ('route_check.json', 'nav2_planner_check.json'):
        path = reports / report
        if path.exists():
            shutil.copy2(path, backup / report)
            path.write_text(json.dumps(dict(result='STALE', map_sha256=new_hash, reason='Image imported; rerun planning checks'), indent=2) + '\n')
    maps.validate(name)
    return f'已导入；备份位于 {backup}。请重新验证规划。'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name')
    parser.add_argument('image')
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()
    try:
        print(import_image(args.name, args.image, args.check_only))
    except (ValueError, OSError, KeyError) as error:
        parser.exit(1, f'导入失败：{error}\n')
