import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_tfg_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hp',
    maintainer_email='hp@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'camera_reader = robot_tfg_pkg.camera_reader:main',
		'arduino_bridge = robot_tfg_pkg.arduino_bridge:main',
		'controller_node = robot_tfg_pkg.controller_node:main',
		'esp32_bridge = robot_tfg_pkg.esp32_bridge:main',
		'odometria = robot_tfg_pkg.odometria_node:main',
        'radar_scanner = robot_tfg_pkg.radar_scanner:main',
        'high_level_controller = robot_tfg_pkg.high_level_controller:main',
        ],
    },
)
