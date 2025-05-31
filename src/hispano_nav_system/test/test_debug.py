import os
import pytest
import importlib.util
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node

# --- IMPORTACIÓN DEL SCRIPT DE LANZAMIENTO ---
def import_launch_file():
    current_dir = Path(__file__).parent
    package_dir = current_dir.parent
    launch_file_path = package_dir / "launch" / "hispano_nav_system.launch.py"
    
    if not launch_file_path.exists():
        pytest.skip(f"Archivo de lanzamiento no encontrado en: {launch_file_path}")
    
    spec = importlib.util.spec_from_file_location("launch_module", launch_file_path)
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)
    return launch_module

launch_module = import_launch_file()

# --- FUNCIONES AUXILIARES ---
def get_node_parameters(node):
    return getattr(node, '_Node__parameters', [])

def get_node_name(node):
    return getattr(node, '_Node__node_name', None)

def test_debug_parameters_structure():
    """Test de diagnóstico para ver la estructura real de los parámetros"""
    ld = launch_module.generate_launch_description()
    
    for entity in ld.entities:
        if isinstance(entity, Node):
            node_name = get_node_name(entity)
            params = get_node_parameters(entity)
            
            print(f"\n=== NODO: {node_name} ===")
            print(f"Tipo de parámetros: {type(params)}")
            print(f"Contenido: {params}")
            
            if isinstance(params, list):
                for i, param in enumerate(params):
                    print(f"  Parámetro {i}: {type(param)} -> {param}")
                    if isinstance(param, dict):
                        print(f"    Claves: {list(param.keys())}")
                        if 'use_sim_time' in param:
                            print(f"    use_sim_time = {param['use_sim_time']}")
                    elif isinstance(param, str):
                        print(f"    Archivo: {os.path.basename(param)}")
    
    # Este test siempre pasa, solo es para debug
    assert True