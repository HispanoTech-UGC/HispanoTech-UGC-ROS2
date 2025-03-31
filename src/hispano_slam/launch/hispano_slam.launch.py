"""@file hispanotech_slam.launch.py
@brief Lanzamiento principal del sistema SLAM HispanoTech
@details Configura y ejecuta los nodos de Cartographer para mapeo y localización
         en entornos industriales. Incluye:
         - Nodo principal de Cartographer
         - Generación de mapa de ocupación
         - Configuración de tiempo real/simulación

@mainpage Launch Configuration
@section hispano_slam_launch Configuración de Lanzamiento
Paquete ROS 2 para despliegue del sistema SLAM en robots HispanoTech.
"""

from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    """@brief Función principal de configuración del lanzamiento
    @details Configura los parámetros y nodos necesarios para el SLAM:
             - Control de tiempo simulado/real
             - Nodo principal de Cartographer
             - Generador de mapa de ocupación
    @return LaunchDescription Configuración completa del lanzamiento
    """
    
    # Configuración de tiempo de simulación (Gazebo)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        # Argumento para tiempo de simulación
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usar reloj de simulación (Gazebo) si es true'
        ),

        # Configuración opcional de logging (descomentar para debug)
        # SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM','1'),
        
        # Nodo principal de Cartographer
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', 
                get_package_share_directory('hispano_slam') + '/config',  # Ruta del paquete
                '-configuration_basename','hispano_slam.lua'  # Archivo de configuración
            ],
            name='cartographer_node',
            emulate_tty=True,
            remappings=[
                ('scan', '/hispano/lidar/scan')  # Remapeo tópico del láser
            ]
        ),
        
        # Generador de mapa de ocupación
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'resolution': 0.05},  # 5cm por celda
                {'publish_period_sec': 1.0}  # Actualización cada 1 segundo
            ],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
            name='occupancy_grid',
            remappings=[
                ('map', '/hispano/navigation/map')  # Topico personalizado del mapa
            ]
        ),
    ])