"""@file turtlebot3_hispano_world.launch.py
@brief Lanzamiento del entorno de simulación HispanoTech para TurtleBot3
@details Configura un entorno Gazebo personalizado con:
         - Mundo virtual HispanoTech
         - Modelos personalizados del robot
         - Integración con paquetes estándar de TurtleBot3
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# Configuración del modelo TurtleBot3 desde variable de entorno
TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']  # burger, waffle, o modelo personalizado

def generate_launch_description():
    """@brief Configuración principal del lanzamiento
    @details Incluye:
             - Servidor Gazebo con mundo personalizado
             - Cliente Gazebo para visualización
             - Publicador de estado del robot
             - Carga de modelos personalizados
    @return LaunchDescription Entorno de simulación completo
    """
    
    # Rutas de archivos configurables
    world_file_name = 'world/' + TURTLEBOT3_MODEL + '_office.world'  # Mundo específico por modelo
    urdf_file_name = 'urdf/turtlebot3_' + TURTLEBOT3_MODEL + '_pi.urdf'  # Modelo URDF del robot
    
    # Configuración de paths para recursos HispanoTech
    pkg_share = FindPackageShare(package='hispanotech_map').find('hispanotech_map')
    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path  # Ruta para modelos personalizados
    
    # Parámetros de ejecución
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = os.path.join(pkg_share, world_file_name)  # Ruta completa al mundo
    urdf = os.path.join(pkg_share, urdf_file_name)  # Modelo URDF del robot

    # Directorios de lanzamiento estándar
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        # Servidor Gazebo con mundo HispanoTech
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),  # Mundo personalizado
        ),

        # Cliente Gazebo (interfaz gráfica)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        # Sistema de estado del robot (URDF + TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),  # Tiempo simulado
        ),

        # Opcional: Spawn del robot en el mundo
        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
        #     arguments=['-entity', TURTLEBOT3_MODEL, '-file', urdf],
        #     output='screen'
        # )
    ])

"""@mainpage Configuración de Simulación
@section Componentes Principales
1. Entorno Gazebo con modelos HispanoTech
2. Robot TurtleBot3 configurado para entornos industriales
3. Integración con paquetes estándar:
   - turtlebot3_gazebo
   - gazebo_ros

@section Características Especiales
- Modelos 3D personalizados para entornos de fábrica
- Configuración de sensores adaptada a necesidades industriales
- Mundo virtual con características específicas HispanoTech
"""