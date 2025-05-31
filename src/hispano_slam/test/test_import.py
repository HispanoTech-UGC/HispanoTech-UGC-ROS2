import importlib
import pytest

def test_import_hispano_slam():
    try:
        importlib.import_module('hispano_slam')
    except ImportError:
        pytest.fail('No se pudo importar el paquete hispano_slam')
