"""
@file test_launch_route_follower.py
@brief Test de lanzamiento para el nodo route_follower del sistema de navegación HispanoTech-UGC-ROS2.

Este archivo utiliza launch_testing para verificar que el nodo route_follower se lanza correctamente.

@author HispanoTech-UGC
@date 2025-05-31
"""
import os
import pytest
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.util import KeepAliveProc
from launch_testing.asserts import assertInStdout
import launch_testing
import launch_testing.actions

@pytest.mark.launch_test
def generate_test_description():
    """
    @brief Genera la descripción de lanzamiento para el test.
    @return LaunchDescription y diccionario vacío para launch_testing.
    """
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    launch_file = os.path.join(pkg_dir, 'launch', 'hispano_nav_system.launch.py')
    assert os.path.exists(launch_file), f"Launch file not found: {launch_file}"
    
    return LaunchDescription([
        Node(
            package='hispano_nav_system',
            executable='route_follower',
            name='route_follower',
            output='screen',
        ),
        KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), {}

def test_route_follower_starts(proc_output):
    """
    @brief Verifica que el nodo route_follower se inicia correctamente.
    @param proc_output Salida del proceso de lanzamiento.
    """
    assert proc_output.wait_for("route_follower", timeout=20)
