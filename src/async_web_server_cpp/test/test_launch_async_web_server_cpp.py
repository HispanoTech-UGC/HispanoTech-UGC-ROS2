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
            package='async_web_server_cpp',
            executable='async_web_server_cpp',
            name='async_web_server_cpp',
            output='screen',
        ),
        KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), {}

def test_async_web_server_cpp_starts(proc_output):
    assert proc_output.wait_for("async_web_server_cpp", timeout=10)
