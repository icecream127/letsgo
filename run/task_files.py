"""Files shared by the task editor, localization and navigation."""
import hashlib
from pathlib import Path

import yaml

PROJECT_NAME = 'competition_tasks.json'
GOALS_NAME = 'nav2_goals.json'


def map_fingerprint(map_yaml):
    path = Path(map_yaml).resolve()
    data = path.read_bytes()
    image = path.parent / yaml.safe_load(data)['image']
    return hashlib.sha256(data + image.read_bytes()).hexdigest()
