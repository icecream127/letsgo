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


def surface_cloud(elevation):
    x, y = np.meshgrid(np.arange(.025, 4., .05), np.arange(.025, 1., .05))
    return np.column_stack((x.ravel(), y.ravel(), elevation(x).ravel()))


def terrain_pixel(image, origin, x, y, resolution=.05):
    ix, iy = np.floor((np.array([x, y]) - origin[:2]) / resolution).astype(int)
    return image[-1 - iy, ix]


def test_terrain_ramp_platform_wall_and_unknown():
    floor = surface_cloud(lambda x: np.clip((x - 1.) * .25, 0., .5))
    # A vertical obstacle on top of the raised platform must remain occupied.
    wall = np.array([[3.625, y, .5 + h] for y in np.arange(.025, 1., .05)
                     for h in np.arange(.025, 1., .025)])
    points = np.vstack((floor, wall))
    image, origin, info = maps.project_terrain(points, .05, 0., .05, .1, 2., .2,
                                               seed_x=.3, seed_y=.5)
    assert terrain_pixel(image, origin, .525, .525) == 254
    assert terrain_pixel(image, origin, 2.525, .525) == 254
    assert terrain_pixel(image, origin, 3.425, .525) == 254
    assert terrain_pixel(image, origin, 3.625, .525) == 0
    assert terrain_pixel(image, origin, -.1, -.1) == 205
    assert info['ground_elevation_range'][1] == pytest.approx(.5)
    flat, flat_origin = maps.project_cloud(points, .05, 0., .05, .1, 2., .2)
    assert terrain_pixel(flat, flat_origin, 2.525, .525) == 0


@pytest.mark.parametrize('step, expected', [(.06, 254), (.10, 254), (.12, 0), (.15, 0), (.25, 0)])
def test_terrain_step_limit_and_disconnected_high_surface(step, expected):
    points = surface_cloud(lambda x: np.where(x < 2., 0., step))
    image, origin, _ = maps.project_terrain(points, .05, 0., .05, .1, 2., .2,
                                           seed_x=.3, seed_y=.5)
    assert terrain_pixel(image, origin, 3.425, .525) == expected


def test_terrain_steep_slope_missing_seed_and_invalid_parameters():
    points = surface_cloud(lambda x: np.maximum(x - 1., 0.) * .8)
    image, origin, _ = maps.project_terrain(points, .05, 0., .05, .1, 2., .2,
                                           seed_x=.3, seed_y=.5)
    assert terrain_pixel(image, origin, 2.025, .525) == 0
    with pytest.raises(ValueError, match='种子'):
        maps.project_terrain(points, .05, -2., .05, .1, 2., .2)
    for kwargs in ({'max_slope': 90.}, {'max_step': -1.}, {'seed_x': float('nan')},
                   {'terrain_resolution': .01}, {'ground_quantile': .9}):
        with pytest.raises(ValueError):
            maps.project_terrain(points, .05, 0., .05, .1, 2., .2, **kwargs)


def test_terrain_15cm_corridor_with_ramp_is_free():
    points = surface_cloud(lambda x: np.clip((x - 1.) * .2, 0., .15))
    image, origin, info = maps.project_terrain(points, .05, 0., .05, .1, 2., .2,
                                               seed_x=.3, seed_y=.5)
    assert terrain_pixel(image, origin, 1.225, .525) == 254
    assert terrain_pixel(image, origin, 2.525, .525) == 254
    assert info['ground_elevation_range'][1] == pytest.approx(.15)


@pytest.mark.parametrize('angle, expected', [(12., 254), (20., 0), (30., 0)])
def test_terrain_user_slope_limit(angle, expected):
    points = surface_cloud(lambda x: np.maximum(x - 1., 0.) * np.tan(np.radians(angle)))
    image, origin, _ = maps.project_terrain(points, .05, 0., .05, .1, 2., .2,
                                           seed_x=.3, seed_y=.5)
    assert terrain_pixel(image, origin, 1.525, .525) == expected


def test_terrain_artifact_validation_and_backup(tmp_path, monkeypatch):
    monkeypatch.setattr(maps, 'PROJECT', tmp_path)
    directory = maps.map_directory('terrain')
    directory.mkdir(parents=True)
    write_pcd(directory / 'cloud.pcd', surface_cloud(lambda x: np.clip((x - 1.) * .25, 0., .5)), True)
    args = SimpleNamespace(name='terrain', force=False, method='terrain', resolution=.05,
                           ground_z=0., ground_tolerance=.05, min_height=.1, max_height=2.,
                           padding=.2, terrain_resolution=.2, ground_quantile=.05,
                           max_step=.08, max_slope=25., seed_x=.3, seed_y=.5, seed_radius=.5)
    maps.project(args)
    old_image = (directory / 'map.pgm').read_bytes()
    args.force = True
    maps.project(args)
    assert maps.validate('terrain') == directory / 'map.yaml'
    backups = list((directory / 'backups').glob('*/map.pgm'))
    assert len(backups) == 1
    assert backups[0].read_bytes() == old_image


def test_external_source_bounds_and_stale_source(tmp_path, monkeypatch):
    monkeypatch.setattr(maps, 'PROJECT', tmp_path)
    source = maps.map_directory('source_room')
    output = maps.map_directory('competition')
    source.mkdir(parents=True)
    output.mkdir()
    write_pcd(source / 'cloud.pcd', surface_cloud(lambda x: np.minimum(x * .1, .15)), True)
    # A different PCD in the output directory must never be read or modified.
    (output / 'cloud.pcd').write_bytes(b'unrelated raw data')
    args = SimpleNamespace(name='competition', source_map='source_room', force=False,
                           method='terrain', resolution=.05, ground_z=0., ground_tolerance=.05,
                           min_height=.1, max_height=2., padding=.2, terrain_resolution=.2,
                           ground_quantile=.05, max_step=.1, max_slope=35., seed_x=.3,
                           seed_y=.5, seed_radius=.5, bounds=[0., 0., 3., 1.])
    maps.project(args)
    assert maps.validate('competition') == output / 'map.yaml'
    import json
    metadata = json.loads((output / 'projection.json').read_text())
    assert metadata['source'] == '../source_room/cloud.pcd'
    assert (metadata['width'], metadata['height']) == (60, 20)
    assert metadata['origin'] == [0., 0., 0.]
    assert (output / 'cloud.pcd').read_bytes() == b'unrelated raw data'
    with (source / 'cloud.pcd').open('ab') as stream:
        stream.write(b'changed')
    with pytest.raises(ValueError, match='cloud.pcd'):
        maps.validate('competition')


def test_single_high_return_does_not_block_observed_floor():
    floor = surface_cloud(lambda x: np.zeros_like(x))
    points = np.vstack((floor, [[1.525, .525, .4]]))
    image, origin, _ = maps.project_terrain(points, .05, 0., .05, .1, 2., .2,
                                           seed_x=.3, seed_y=.5)
    assert terrain_pixel(image, origin, 1.525, .525) == 254


def test_dense_vertical_obstacle_still_blocks_floor():
    floor = surface_cloud(lambda x: np.zeros_like(x))
    points = np.vstack((floor, [[1.525, .525, z] for z in [.2, .3, .4, .5]]))
    image, origin, _ = maps.project_terrain(points, .05, 0., .05, .1, 2., .2,
                                           seed_x=.3, seed_y=.5)
    assert terrain_pixel(image, origin, 1.525, .525) == 0


def test_confirmed_slatted_region_preserves_edges_objects_and_outside():
    x, y = np.meshgrid(np.arange(.025, 4., .05), np.arange(.025, 2., .05))
    z = np.where((x % .5) < .15, -.2, .2)  # real gaps, not noise
    cloud = np.column_stack((x.ravel(), y.ravel(), z.ravel()))
    obstacle = np.array([[2.025, .525, h] for h in np.linspace(.5, 1., 12)])
    image = np.full((40, 80), 205, dtype=np.uint8)
    profile = {'schema': 1, 'confirmation': 'Robot tested on this specific slatted terrain',
               'regions': [{'name': 'slats', 'x_range': [1., 3.],
                            'normal_equation': [0., 1., 0.], 'normal_range': [0., 1.],
                            'surface_z_band': [.15, .25], 'protect_sides': True}]}
    result, reports = maps.apply_confirmed_regions(image, [0., 0., 0.],
                                                   np.vstack((cloud, obstacle)), .05, profile, 2.)
    assert terrain_pixel(result, [0., 0., 0.], 1.525, .525) == 254  # confirmed gap
    assert terrain_pixel(result, [0., 0., 0.], 2.025, .525) == 0    # actual object
    assert terrain_pixel(result, [0., 0., 0.], 1.525, .025) == 0   # lateral edge
    assert terrain_pixel(result, [0., 0., 0.], .525, .525) == 205  # no global filling
    assert reports[0]['gap_width_limit'] is None
    assert np.array_equal(result[:, :20], image[:, :20])


def test_confirmed_region_requires_surface_evidence():
    profile = {'schema': 1, 'confirmation': 'specific region tested', 'regions': [
        {'name': 'slats', 'x_range': [0., 1.], 'normal_equation': [0., 1., 0.],
         'normal_range': [0., 1.], 'surface_z_band': [.15, .25]}]}
    with pytest.raises(ValueError, match='insufficient'):
        maps.apply_confirmed_regions(np.full((20, 20), 205, dtype=np.uint8), [0., 0., 0.],
                                     np.array([[.5, .5, -.2]]), .05, profile, 2.)
