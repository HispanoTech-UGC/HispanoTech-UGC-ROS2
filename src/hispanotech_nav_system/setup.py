import os
from glob import glob
from setuptools import setup
from setuptools import setup, find_packages


package_name = 'hispanotech_nav_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['hispanotech_nav_system', 'hispanotech_nav_system.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.pgm')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asun',
    maintainer_email='asun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_pose_pub = hispanotech_nav_system.initial_pose_pub:main', #añadir
            'navigate_to_pose_client = hispanotech_nav_system.navigate_to_pose_client:main',
            'navigate_to_pose_server = hispanotech_nav_system.navigate_to_pose_server:main',                
            ],

















    },
)
