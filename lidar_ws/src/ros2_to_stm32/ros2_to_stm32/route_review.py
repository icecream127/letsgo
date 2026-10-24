import hashlib
import json
import math
from pathlib import Path


HANDLERS = {
    '木桥A': 'handle_bridge_A', '木桥B': 'handle_bridge_B',
    '大斜坡': 'handle_big_slope', '限高栏': 'handle_limit_pole',
    '沙坑': 'handle_bunker', '绕杆': 'handle_pole',
    '高墙': 'handle_hill_wall', '台阶': 'handle_step',
}


def contains(region, x_pos, y_pos):
    return (region['x_min'] <= x_pos < region['x_max']
            and region['y_min'] <= y_pos < region['y_max'])


def overlaps(first, second):
    return (max(first['x_min'], second['x_min']) < min(first['x_max'], second['x_max'])
            and max(first['y_min'], second['y_min']) < min(first['y_max'], second['y_max']))


def load_review(path):
    raw = Path(path).read_bytes()
    data = json.loads(raw)
    if not isinstance(data, dict) or not isinstance(data.get('zones'), dict):
        raise ValueError('地图必须包含 zones 对象')
    if not isinstance(data.get('zone'), dict):
        raise ValueError('地图必须包含 zone 对象')
    if set(data['zones']) != set(HANDLERS):
        raise ValueError('地图必须包含八个已支持的障碍区域')
    issues = []
    for section in ('zones', 'zone'):
        for name, region in data[section].items():
            fields = ['x_min', 'x_max', 'y_min', 'y_max']
            if section == 'zones':
                fields += ['begin_x', 'begin_y']
            if not isinstance(region, dict) or any(
                    not isinstance(region.get(field), (int, float))
                    or isinstance(region[field], bool)
                    or not math.isfinite(region[field]) for field in fields):
                raise ValueError(f'{name} 坐标缺失或不是有限数值')
            if region['x_min'] >= region['x_max'] or region['y_min'] >= region['y_max']:
                raise ValueError(f'{name} 区域上下界无效')
            if section == 'zones':
                if region.get('func_name') != HANDLERS[name]:
                    raise ValueError(f'{name} 处理函数不匹配')
                if not contains(region, region['begin_x'], region['begin_y']):
                    issues.append(f'{name}：参考原点在所属区域外，需核对航点偏移，不能自动判错或翻转符号')
            elif region.get('func_name') not in data['zones']:
                raise ValueError(f'{name} 必须指定有效的目标障碍名称')
    regions = list(data['zones'].items()) + list(data['zone'].items())
    for index, (name, region) in enumerate(regions):
        for other_name, other in regions[index + 1:]:
            if overlaps(region, other):
                issues.append(f'{name} 与 {other_name} 重叠：障碍优先，其次按文件顺序首次匹配')
    return data, hashlib.sha256(raw).hexdigest(), issues


def select_region(data, x_pos, y_pos):
    for name, region in data['zones'].items():
        if contains(region, x_pos, y_pos):
            return name, region
    for name, region in data['zone'].items():
        if contains(region, x_pos, y_pos):
            target_name = region['func_name']
            return target_name, data['zones'][target_name]
    return None, None
