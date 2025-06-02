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
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
        ),
        KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), {}

def test_web_video_server_starts(proc_output):
    assert proc_output.wait_for("web_video_server", timeout=10)
