import os
import pytest

def test_map_file_exists():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    map_file = os.path.join(pkg_dir, 'config', 'hispano_map.yaml')
    assert os.path.exists(map_file), f"Map file not found: {map_file}"

def test_nav_params_file_exists():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    nav2_yaml_path = os.path.join(pkg_dir, 'config', 'hispano_nav_params.yaml')
    assert os.path.exists(nav2_yaml_path), f"Nav2 params file not found: {nav2_yaml_path}"
