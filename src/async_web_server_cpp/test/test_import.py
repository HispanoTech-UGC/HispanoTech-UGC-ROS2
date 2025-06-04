import importlib
import pytest

def test_import_async_web_server_cpp():
    try:
        importlib.import_module('async_web_server_cpp')
    except ImportError:
        pytest.fail('No se pudo importar el paquete async_web_server_cpp')
