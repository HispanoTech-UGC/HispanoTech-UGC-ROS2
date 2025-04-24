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
        (os.path.join('share', package_name, 'map'), glob('map/*.pgm')),#incluir
        (os.path.join('share', package_name, 'map'), glob('map/*.yaml')),#incluir
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),#incluir
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),#incluir
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),#incluir

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asperez@upv.es',
    maintainer_email='asperez@upv.es',
    description='TODO: Package description',
    license='TODO: License declaration',
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