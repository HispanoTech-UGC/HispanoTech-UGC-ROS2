"""
test_launch_world.py
Test unitario para verificar el correcto funcionamiento del archivo de lanzamiento
turtlebot3_hispano_world.launch.py del paquete hispano_map
"""

import os
import sys
import pytest
import tempfile
import importlib.util
from unittest import mock
from pathlib import Path

# Importamos el archivo de lanzamiento directamente mediante importlib
def import_launch_file():
    """Importa el archivo de lanzamiento usando importlib"""
    # Determina la ruta al archivo de lanzamiento
    current_dir = Path(__file__).parent
    package_dir = current_dir.parent
    launch_file_path = package_dir / "launch" / "turtlebot3_hispano_world.launch.py"
    
    if not launch_file_path.exists():
        pytest.skip(f"Archivo de lanzamiento no encontrado en: {launch_file_path}")
    
    # Importa el módulo usando importlib
    spec = importlib.util.spec_from_file_location("turtlebot3_hispano_world", launch_file_path)
    turtlebot3_hispano_world = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(turtlebot3_hispano_world)
    return turtlebot3_hispano_world

# Cargamos el módulo a testear
turtlebot3_hispano_world = import_launch_file()

# Importaciones adicionales necesarias para testing
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration


@pytest.fixture
def set_test_environment():
    """Configura el entorno para las pruebas, estableciendo la variable de entorno TURTLEBOT3_MODEL"""
    old_env = None
    if 'TURTLEBOT3_MODEL' in os.environ:
        old_env = os.environ['TURTLEBOT3_MODEL']
    
    # Configuramos el modelo para el test
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    
    yield
    
    # Restauramos la variable de entorno a su valor original
    if old_env is not None:
        os.environ['TURTLEBOT3_MODEL'] = old_env
    else:
        if 'TURTLEBOT3_MODEL' in os.environ:
            del os.environ['TURTLEBOT3_MODEL']


def test_generate_launch_description(set_test_environment):
    """Verifica que la función generate_launch_description devuelve un LaunchDescription válido"""
    launch_description = turtlebot3_hispano_world.generate_launch_description()
    assert isinstance(launch_description, LaunchDescription)
    
    # Verificamos que el LaunchDescription contiene al menos un elemento
    assert len(launch_description.entities) > 0


def test_launch_file_includes(set_test_environment):
    """Verifica que los archivos de lanzamiento necesarios están incluidos"""
    launch_description = turtlebot3_hispano_world.generate_launch_description()
    
    # Contamos el número de IncludeLaunchDescription
    include_count = sum(1 for entity in launch_description.entities if isinstance(entity, IncludeLaunchDescription))
    
    # Esperamos al menos 3 IncludeLaunchDescription (gzserver, gzclient, robot_state_publisher)
    assert include_count >= 3


@mock.patch('os.path.join')
@mock.patch('ament_index_python.packages.get_package_share_directory')
@mock.patch.object(turtlebot3_hispano_world, 'FindPackageShare')
def test_resource_paths(mock_find_package, mock_get_package, mock_path_join, set_test_environment):
    """Verifica que las rutas a los recursos se construyen correctamente"""
    # Configuramos los mocks para simular las rutas
    mock_pkg_share = mock.MagicMock()
    mock_pkg_share.find.return_value = '/fake/path/hispano_map'
    mock_find_package.return_value = mock_pkg_share
    
    mock_get_package.return_value = '/fake/path/turtlebot3_gazebo'
    
    # El mock de os.path.join simplemente une las cadenas con '/'
    mock_path_join.side_effect = lambda *args: '/'.join(args)
    
    # Llamamos a la función
    turtlebot3_hispano_world.generate_launch_description()
    
    # Verificamos que se accede al paquete hispano_map
    mock_find_package.assert_called_with(package='hispano_map')
    
    # Verificamos que se configura la variable de entorno GAZEBO_MODEL_PATH
    assert 'GAZEBO_MODEL_PATH' in os.environ
    assert os.environ['GAZEBO_MODEL_PATH'] == '/fake/path/hispano_map/models'



@pytest.mark.parametrize("model", ["burger", "waffle"])
def test_different_robot_models(model):
    """Verifica que se puede configurar diferentes modelos de robot"""
    # Guardamos el valor original
    old_env = os.environ.get('TURTLEBOT3_MODEL', None)
    
    try:
        # Configuramos el modelo para el test
        os.environ['TURTLEBOT3_MODEL'] = model
        
        # Verificamos que no hay excepciones al generar el launch description
        launch_description = turtlebot3_hispano_world.generate_launch_description()
        assert isinstance(launch_description, LaunchDescription)
    finally:
        # Restauramos la variable de entorno a su valor original
        if old_env is not None:
            os.environ['TURTLEBOT3_MODEL'] = old_env
        else:
            del os.environ['TURTLEBOT3_MODEL']


def test_missing_environment_variable():
    """Verifica el comportamiento cuando falta la variable de entorno TURTLEBOT3_MODEL"""
    # Guardamos el valor original
    old_env = os.environ.get('TURTLEBOT3_MODEL', None)
    
    if 'TURTLEBOT3_MODEL' in os.environ:
        del os.environ['TURTLEBOT3_MODEL']
    
    try:
        # Debería lanzar una excepción cuando la variable no está definida
        with pytest.raises(KeyError):
            turtlebot3_hispano_world.generate_launch_description()
    finally:
        # Restauramos la variable de entorno a su valor original
        if old_env is not None:
            os.environ['TURTLEBOT3_MODEL'] = old_env