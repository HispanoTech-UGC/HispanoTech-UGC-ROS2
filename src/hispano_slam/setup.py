"""@file setup.py
@brief Configuración de empaquetado ROS 2 para el sistema SLAM HispanoTech
@details Define la estructura del paquete y los recursos a instalar
         Incluye:
         - Configuraciones Cartographer
         - Mapas predefinidos
         - Archivos de lanzamiento
         - Configuraciones de visualización RViz
"""

from setuptools import setup
import os
from glob import glob

# Configuración principal del paquete
package_name = 'hispano_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Estructura estándar de paquetes ROS 2
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Recursos específicos del proyecto
        ## Mapas base (formato PGM+YAML)
        (os.path.join('share', package_name, 'map'), glob('map/*.pgm')),
        (os.path.join('share', package_name, 'map'), glob('map/*.yaml')),
        
        ## Configuraciones de visualización RViz
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        
        ## Archivos de lanzamiento
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        ## Configuraciones Cartographer (.lua)
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    
    # Metadatos del paquete
    maintainer='Equipo HispanoTech',
    maintainer_email='smariba@epsg.upv.es',
    description='Sistema de SLAM para robots industriales HispanoTech',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Sin ejecutables directos - todo se maneja mediante nodos Cartographer
        ],
    },
)

"""@mainpage Estructura del Paquete
@section Estructura de Directorios
- /map: Mapas base en formato PGM+YAML
- /rviz: Configuraciones de visualización
- /launch: Archivos de lanzamiento
- /config: Parámetros Cartographer

@section Instalación
El paquete sigue el estándard ROS 2 para instalación mediante colcon
"""