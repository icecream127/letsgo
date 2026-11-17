import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('nav_goals', Path(__file__).resolve().parents[1] / 'run/nav_goals.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

class GoalsTest(unittest.TestCase):
    def test_success_order(self):
        visited = []
        self.assertTrue(m.execute_sequence([1, 2, 3], lambda g: visited.append(g) or True))
        self.assertEqual(visited, [1, 2, 3])

    def test_failure_stops(self):
        visited = []
        self.assertFalse(m.execute_sequence([1, 2, 3], lambda g: visited.append(g) or g != 2))
        self.assertEqual(visited, [1, 2])

    def test_input_validation(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / 'goals.json'
            for bad in ('{"frame_id":"odom","goals":[{}]}', '{"frame_id":"map","goals":[]}', '{"frame_id":"map","goals":[{"x":NaN,"y":0,"yaw":0}]}'):
                p.write_text(bad)
                with self.assertRaises(ValueError): m.load_goals(p)
            p.write_text('{"frame_id":"map","goals":[{"x":1,"y":2,"yaw":0}]}')
            self.assertEqual(m.load_goals(p)[0]['x'], 1)

if __name__ == '__main__': unittest.main()
