import importlib
import pytest

def test_import_provide_hispano_map():
    try:
        importlib.import_module('provide_hispano_map')
    except ImportError:
        pytest.fail('No se pudo importar el paquete provide_hispano_map')
