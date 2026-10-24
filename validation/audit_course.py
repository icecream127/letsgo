import argparse
import ast
import json
from pathlib import Path
import sys
from types import SimpleNamespace

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / 'lidar_ws/src/ros2_to_stm32'))
from ros2_to_stm32.route_review import load_review, select_region


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--controller', type=Path)
    arguments = parser.parse_args()
    package = PROJECT / 'lidar_ws/src/ros2_to_stm32'
    map_path = package / 'config/map_zones_config_2.json'
    data, fingerprint, issues = load_review(map_path)
    controller_path = arguments.controller or package / 'ros2_to_stm32/task_to_do_unitree.py'
    source = ast.parse(controller_path.read_text(encoding='utf-8'))
    controller = next(node for node in source.body if isinstance(node, ast.ClassDef))
    methods = {node.name: node for node in controller.body if isinstance(node, ast.FunctionDef)}
    targets = []
    outside = []
    for name, region in data['zones'].items():
        method = methods[region['func_name']]
        assignment = next(node for node in method.body if isinstance(node, ast.Assign)
                          and any(isinstance(target, ast.Name) and target.id == 'Waypoints'
                                  for target in node.targets))
        values = dict(x=0.0, y=0.0, yaw=0.0, x_begin=region['begin_x'],
                      y_begin=region['begin_y'], self=SimpleNamespace(zones=data['zones']))
        for index, expression in enumerate(assignment.value.elts):
            dynamic = any(isinstance(node, ast.Name) and node.id in ('x', 'y')
                          for part in expression.elts[:2] for node in ast.walk(part))
            waypoint = eval(compile(ast.Expression(expression), '<route audit>', 'eval'),
                            {'__builtins__': {}}, values)
            global_target = [region['begin_x'] + waypoint[0], region['begin_y'] + waypoint[1]]
            owner, _ = select_region(data, *global_target)
            item = dict(zone=name, waypoint=index + 1,
                        expression=ast.unparse(expression), dynamic_xy=dynamic,
                        global_xy=None if dynamic else global_target,
                        target_dispatch=None if dynamic else owner)
            targets.append(item)
            if not dynamic and owner is None:
                outside.append(f'{name} 航点 {index + 1}: {global_target} 不在任何调度区域')
    report = dict(map=str(map_path), controller=str(controller_path), map_sha256=fingerprint,
                  geometry_issues=issues, fixed_targets_outside_map=outside,
                  waypoints=targets,
                  limitations='仅检查固定目标和区域关系；动态航点、区域切换、实物尺寸和动作效果须另验。未发布ROS消息。')
    if arguments.output:
        arguments.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(f'地图: {map_path}\nSHA256: {fingerprint}\n八个障碍，共 {len(targets)} 个航点')
    for issue in issues + outside:
        print(issue)
    print('本命令不写入审核值、不启用运动；发现问题必须修正上位机航点/调度后复验。')
    if arguments.check and outside:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
