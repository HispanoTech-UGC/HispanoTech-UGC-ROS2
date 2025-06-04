import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    pkg_dir = get_package_share_directory('hispano_nav_system')

    nav2_yaml_path = os.path.join(pkg_dir, 'config', 'hispano_nav_params.yaml')
    map_file = os.path.join(pkg_dir, 'config', 'hispano_map.yaml')
    rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'rviz_hispano_slam.rviz')
    ruta_txt = os.path.join(pkg_dir, 'save_paths', 'ruta_guardada.txt')

    nav2_params = load_yaml(nav2_yaml_path)

    def with_common_params(node_name, extra_params=None):
        # Extrae solo los parámetros de ese nodo si existen
        node_entry = nav2_params.get(node_name, {})
        node_params = node_entry.get('ros__parameters', {}).copy() if 'ros__parameters' in node_entry else {}
        node_params['use_sim_time'] = True
        node_params['config_file_path'] = nav2_yaml_path  # Para los tests
        if extra_params:
            node_params.update(extra_params)
        return [node_params]

    return LaunchDescription([
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=with_common_params('waypoint_follower', {'ruta_waypoints': ruta_txt})
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=with_common_params('map_server', {'yaml_filename': map_file})
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=with_common_params('amcl')
        ),
        Node(
            package='hispano_nav_system',
            executable='route_follower',
            name='route_follower',
            output='screen',
            parameters=with_common_params('route_follower', {'ruta_archivo': ruta_txt})
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=with_common_params('planner_server')
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=with_common_params('controller_server')
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=with_common_params('bt_navigator')
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=with_common_params('recoveries_server')
        ),
        Node(
            package='hispano_nav_system',
            executable='route_recorder',
            name='route_recorder',
            output='screen',
            parameters=with_common_params('route_recorder')
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=with_common_params('joy_node')
        ),
        Node(
            package='hispano_nav_system',
            executable='ps3_joy_teleop',
            name='ps3_joy_teleop',
            output='screen',
            parameters=with_common_params('ps3_joy_teleop')
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=with_common_params('lifecycle_manager_pathplanner', {
                'autostart': True,
                'node_names': [
                    'amcl', 'planner_server', 'controller_server',
                    'recoveries_server', 'bt_navigator',
                    'map_server', 'waypoint_follower'
                ]
            })
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=with_common_params('rviz2')
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_camera_link',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_link'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_camera_frame',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_rgb_frame'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_camera_optical_frame',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_rgb_optical_frame'],
            output='screen'
        ),
    ])
