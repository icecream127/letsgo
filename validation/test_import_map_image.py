import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
from PIL import Image
import pytest
from test_maps import write_pcd

spec = importlib.util.spec_from_file_location('import_map_image', Path(__file__).parents[1] / 'run/import_map_image.py')
sys.path.insert(0, str(Path(__file__).parents[1] / 'run'))
importer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(importer)


def prepare(tmp_path, monkeypatch):
    monkeypatch.setattr(importer.maps, 'PROJECT', tmp_path)
    directory = importer.maps.map_directory('room')
    directory.mkdir(parents=True)
    write_pcd(directory / 'cloud.pcd', [[0, 0, 0], [1, 1, 0], [0, 1, .5]])
    importer.maps.project(SimpleNamespace(name='room', force=False, resolution=.1, ground_z=0., ground_tolerance=.05, min_height=.1, max_height=2., padding=.5))
    return directory


def test_import_preserves_geometry_source_and_archives_outside_maps(tmp_path, monkeypatch):
    directory = prepare(tmp_path, monkeypatch)
    before = {n: (directory / n).read_bytes() for n in ['cloud.pcd', 'map.yaml', 'map.pgm']}
    pixels = np.array(Image.open(directory / 'map.pgm'));pixels[0, 0] = 254
    edited = tmp_path / 'edited.pgm';Image.fromarray(pixels).save(edited)
    importer.import_image('room', edited, True)
    assert (directory / 'map.pgm').read_bytes() == before['map.pgm']
    importer.import_image('room', edited)
    assert (directory / 'map.pgm').read_bytes() == edited.read_bytes()
    for name in ['cloud.pcd', 'map.yaml']:
        assert (directory / name).read_bytes() == before[name]
    assert len(list((tmp_path / '.local/map_backups/room').glob('*/map.pgm'))) == 1
    assert not (directory / 'backups').exists()
    importer.maps.validate('room')


def test_invalid_edit_never_replaces_map(tmp_path, monkeypatch):
    directory = prepare(tmp_path, monkeypatch)
    before = (directory / 'map.pgm').read_bytes()
    edited = tmp_path / 'edited.pgm'
    Image.new('L', (2, 2), 254).save(edited)
    with pytest.raises(ValueError):
        importer.import_image('room', edited)
    Image.new('L', Image.open(directory / 'map.pgm').size, 128).save(edited)
    with pytest.raises(ValueError):
        importer.import_image('room', edited)
    assert (directory / 'map.pgm').read_bytes() == before
