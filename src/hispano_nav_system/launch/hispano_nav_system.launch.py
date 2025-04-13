import os
import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    @brief Generate a launch description for the navigation stack.
         
    This function sets up the necessary nodes for robot navigation using Nav2,
    including localization (AMCL), path planning, controllers, recovery behaviors,
    RViz visualization, and map server functionalities.
    
    @return LaunchDescription The configured launch description for Nav2 and related nodes.
    """
    "@brief Path to the Nav2 configuration YAML file containing all navigation parameters."
    nav2_yaml = os.path.join(get_package_share_directory('hispano_nav_system'), 'config', 'hispano_nav_params.yaml')
    "@brief Path to the map YAML file specifying the robot's environment for navigation."
    map_file = os.path.join(get_package_share_directory('hispano_nav_system'), 'config', 'hispano_map.yaml')
    #map_file = os.path.join(get_package_share_directory('hispano_nav_system'), 'config', 'turtlebot3_world.yaml')
    "@brief Path to the RViz configuration file for visualizing navigation behavior."
    rviz_config_dir = os.path.join(get_package_share_directory('hispano_nav_system'), 'rviz', 'rviz_hispano_slam.rviz')
   # urdf = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_burger.urdf')
   # world = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_worlds/burger.model')
    pkg_dir = get_package_share_directory('hispano_nav_system')
    config_dir = os.path.join(pkg_dir, 'config')

    return LaunchDescription([
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}, {'ruta_waypoints': os.path.join(config_dir, 'ruta_guardada.txt')}]
        ),
        
        Node(
            package = 'nav2_map_server',
            executable = 'map_server',
            name = 'map_server',
            output = 'screen',
            parameters=[{'yaml_filename':os.path.join(config_dir, 'hispano_map.yaml')}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),


        Node(
            package = 'nav2_planner',
            executable = 'planner_server',
            name = 'planner_server',
            output = 'screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}]
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}]
        ),
        Node(
            package='hispano_nav_system',
            executable='route_recorder',
            name='route_recorder',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),       
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names':['amcl', 'planner_server', 'controller_server', 'recoveries_server', 'bt_navigator', 'map_server', 'waypoint_follower']}]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        #TRANSFORMADAS CÁMARA
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
        Node(
               package='rviz2',
               executable='rviz2',
               name='rviz2',
               arguments=['-d', rviz_config_dir],
               parameters=[{'use_sim_time': True}],
               output='screen'
        )
    ])