import importlib
import pytest

def test_import_web_video_server():
    try:
        importlib.import_module('web_video_server')
    except ImportError:
        pytest.fail('No se pudo importar el paquete web_video_server')
