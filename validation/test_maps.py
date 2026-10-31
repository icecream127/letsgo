import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location('maps', Path(__file__).parents[1] / 'run/maps.py')
maps = importlib.util.module_from_spec(spec)
spec.loader.exec_module(maps)


def write_pcd(path, points, binary=False):
    # Extra intensity field mirrors the real PCL map layout; x/y/z are not assumed contiguous.
    points = np.asarray(points, dtype='<f4')
    header = (f'VERSION .7\nFIELDS x intensity y z\nSIZE 4 4 4 4\nTYPE F F F F\n'
              f'COUNT 1 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\n'
              f'DATA {"binary" if binary else "ascii"}\n').encode()
    records = np.column_stack((points[:, 0], np.ones(len(points)), points[:, 1:])).astype('<f4')
    body = records.tobytes() if binary else ('\n'.join(' '.join(map(str, row)) for row in records) + '\n').encode()
    path.write_bytes(header + body)


@pytest.mark.parametrize('binary', [False, True])
def test_pcd_roundtrip_with_extra_field(tmp_path, binary):
    path = tmp_path / 'cloud.pcd'
    points = [[1, 2, 3], [-1, -2, -3], [float('nan'), 0, 0]]
    write_pcd(path, points, binary)
    np.testing.assert_allclose(maps.read_pcd(path), points[:2])
    path.write_bytes(path.read_bytes()[:-5])
    with pytest.raises(ValueError):
        maps.read_pcd(path)


def test_projection_orientation_unknown_and_obstacle_priority():
    points = np.array([[0, 0, 0], [0, 0, .5], [1, 1, 0], [2, 2, 4.]])
    image, origin = maps.project_cloud(points, 1., 0., .05, .1, 2., 0.)
    assert origin == [0., 0., 0.]
    np.testing.assert_array_equal(image, [[205, 254], [0, 205]])


def test_missing_ground_and_huge_grid_rejected():
    with pytest.raises(ValueError, match='地面点'):
        maps.project_cloud(np.array([[0, 0, 1.]]), .05, 0., .05, .1, 2., .5)
    with pytest.raises(ValueError, match='16 million'):
        maps.project_cloud(np.array([[0, 0, 0.], [100000, 100000, 0.]]), .05, 0., .05, .1, 2., .5)


def test_map_paths_stale_artifacts_and_no_overwrite(tmp_path, monkeypatch):
    monkeypatch.setattr(maps, 'PROJECT', tmp_path)
    for invalid in ('../escape', '/tmp/escape', '', 'a/b'):
        with pytest.raises(ValueError):
            maps.map_directory(invalid)
    directory = maps.map_directory('room_1')
    directory.mkdir(parents=True)
    write_pcd(directory / 'cloud.pcd', [[0, 0, 0], [1, 1, 0], [0, 1, .5]], True)
    args = SimpleNamespace(name='room_1', force=False, resolution=.1, ground_z=0.,
                           ground_tolerance=.05, min_height=.1, max_height=2., padding=.5)
    maps.project(args)
    assert maps.validate('room_1') == directory / 'map.yaml'
    with pytest.raises(ValueError, match='已存在'):
        maps.project(args)
    with (directory / 'cloud.pcd').open('ab') as stream:
        stream.write(b'changed')
    with pytest.raises(ValueError, match='cloud.pcd'):
        maps.validate('room_1')
    (tmp_path / 'maps' / 'escape').symlink_to(tmp_path, target_is_directory=True)
    with pytest.raises(ValueError):
        maps.map_directory('escape')
