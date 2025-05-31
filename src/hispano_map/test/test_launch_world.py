"""
test_launch_world.py
Test unitario para verificar el correcto funcionamiento del archivo de lanzamiento
`turtlebot3_hispano_world.launch.py` del paquete hispano_map.
"""

import os
import pytest
import importlib.util
from unittest import mock
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription


def import_launch_file():
    """Importa el archivo de lanzamiento usando importlib."""
    current_dir = Path(__file__).parent
    package_dir = current_dir.parent
    launch_file_path = package_dir / "launch" / "turtlebot3_hispano_world.launch.py"
    if not launch_file_path.exists():
        pytest.skip(f"Archivo de lanzamiento no encontrado en: {launch_file_path}")
    spec = importlib.util.spec_from_file_location(
        "turtlebot3_hispano_world", launch_file_path)
    turtlebot3_hispano_world = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(turtlebot3_hispano_world)
    return turtlebot3_hispano_world


turtlebot3_hispano_world = import_launch_file()


@pytest.fixture
def set_test_environment():
    """Configura el entorno para las pruebas, estableciendo la variable de entorno TURTLEBOT3_MODEL."""
    old_env = None
    if 'TURTLEBOT3_MODEL' in os.environ:
        old_env = os.environ['TURTLEBOT3_MODEL']
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    yield
    if old_env is not None:
        os.environ['TURTLEBOT3_MODEL'] = old_env
    else:
        if 'TURTLEBOT3_MODEL' in os.environ:
            del os.environ['TURTLEBOT3_MODEL']


def test_generate_launch_description(set_test_environment):
    """Verifica que la función generate_launch_description devuelve un LaunchDescription válido."""
    launch_description = turtlebot3_hispano_world.generate_launch_description()
    assert isinstance(launch_description, LaunchDescription)
    assert len(launch_description.entities) > 0


def test_launch_file_includes(set_test_environment):
    """Verifica que los archivos de lanzamiento necesarios están incluidos."""
    launch_description = turtlebot3_hispano_world.generate_launch_description()
    include_count = sum(
        1 for entity in launch_description.entities if isinstance(entity, IncludeLaunchDescription)
    )
    assert include_count >= 3


@mock.patch('os.path.join')
@mock.patch('ament_index_python.packages.get_package_share_directory')
@mock.patch.object(turtlebot3_hispano_world, 'FindPackageShare')
def test_resource_paths(mock_find_package, mock_get_package, mock_path_join, set_test_environment):
    """Verifica que las rutas a los recursos se construyen correctamente."""
    mock_pkg_share = mock.MagicMock()
    mock_pkg_share.find.return_value = '/fake/path/hispano_map'
    mock_find_package.return_value = mock_pkg_share
    mock_get_package.return_value = '/fake/path/turtlebot3_gazebo'
    mock_path_join.side_effect = lambda *args: '/'.join(args)
    turtlebot3_hispano_world.generate_launch_description()
    mock_find_package.assert_called_with(package='hispano_map')
    assert 'GAZEBO_MODEL_PATH' in os.environ
    assert os.environ['GAZEBO_MODEL_PATH'] == '/fake/path/hispano_map/models'


@pytest.mark.parametrize("model", ["burger", "waffle"])
def test_different_robot_models(model):
    """Verifica que se puede configurar diferentes modelos de robot."""
    old_env = os.environ.get('TURTLEBOT3_MODEL', None)
    try:
        os.environ['TURTLEBOT3_MODEL'] = model
        launch_description = turtlebot3_hispano_world.generate_launch_description()
        assert isinstance(launch_description, LaunchDescription)
    finally:
        if old_env is not None:
            os.environ['TURTLEBOT3_MODEL'] = old_env
        else:
            if 'TURTLEBOT3_MODEL' in os.environ:
                del os.environ['TURTLEBOT3_MODEL']


def test_missing_environment_variable():
    """Verifica el comportamiento cuando falta la variable de entorno TURTLEBOT3_MODEL."""
    old_env = os.environ.get('TURTLEBOT3_MODEL', None)
    if 'TURTLEBOT3_MODEL' in os.environ:
        del os.environ['TURTLEBOT3_MODEL']
    try:
        with pytest.raises(KeyError):
            turtlebot3_hispano_world.generate_launch_description()
    finally:
        if old_env is not None:
            os.environ['TURTLEBOT3_MODEL'] = old_env
        else:
            if 'TURTLEBOT3_MODEL' in os.environ:
                del os.environ['TURTLEBOT3_MODEL']