import os
import pytest

def test_map_yaml_exists():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    map_file = os.path.join(pkg_dir, 'config', 'hispano_map.yaml')
    assert os.path.exists(map_file), f"Map yaml not found: {map_file}"
