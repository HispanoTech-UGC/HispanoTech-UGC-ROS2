from setuptools import setup

package_name = 'hispano_weapon_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'roboflow',
        'opencv-python',
        'numpy',
    ],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='prebdem@epsg.upv.es',
    description='Nodo ROS 2 que detecta personas y armas con Roboflow',
    license='MIT',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'detector_personas_armas = hispano_weapon_detector.detector_personas_armas:main',
        ],
    },

    data_files=[
        # 1) Instala el marcador dentro de ament_index/resource_index/packages
        ('share/ament_index/resource_index/packages',
            ['resource/hispano_weapon_detector']),
        # 2) Copia el package.xml a share/<package_name>
        ('share/' + package_name, ['package.xml']),
    ],
)
