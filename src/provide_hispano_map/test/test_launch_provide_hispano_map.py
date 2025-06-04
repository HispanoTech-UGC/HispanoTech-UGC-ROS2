import os
import pytest
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.util import KeepAliveProc
import launch_testing
import launch_testing.actions

@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        Node(
            package='provide_hispano_map',
            executable='provide_hispano_map',
            name='provide_hispano_map',
            output='screen',
        ),
        KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), {}

def test_provide_hispano_map_starts(proc_output):
    assert proc_output.wait_for("provide_hispano_map", timeout=10)
