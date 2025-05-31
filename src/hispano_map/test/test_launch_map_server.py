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
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    launch_file = os.path.join(pkg_dir, 'launch', 'launch_world.launch.py')
    assert os.path.exists(launch_file), f"Launch file not found: {launch_file}"
    
    return LaunchDescription([
        Node(
            package='hispano_map',
            executable='map_server',
            name='map_server',
            output='screen',
        ),
        KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), {}

def test_map_server_starts(proc_output):
    assert proc_output.wait_for("map_server", timeout=10)
