import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    @brief Generate a launch description for running nodes in the navigation stack.
    
    This function sets up and launches the nodes required for robot localization
    and map visualization, including the map server, RViz, and a static transform publisher.
    
    @return LaunchDescription The launch description that defines the nodes and their parameters.
    
    """
    "@brief Path to the RViz configuration file."
    rviz_config_dir = os.path.join(get_package_share_directory('provide_hispano_map'), 'rviz', 'rviz_hispano_slam.rviz')
    map_file = os.path.join(get_package_share_directory('provide_hispano_map'), 'map', 'warehouse.yaml')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename': map_file}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        )
    ])
