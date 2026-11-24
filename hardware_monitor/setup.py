import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'hardware_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Instala los archivos launch para poder usar 'ros2 launch'
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco Antonio Ramos Sesma',
    maintainer_email='marco.ramos9056@alumnos.udg.mx',
    description='Actividad 3: emulador, monitor y operador de hardware en ROS 2',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hardware_emulator = hardware_monitor.hardware_emulator:main',
            'monitor_node = hardware_monitor.monitor_node:main',
            'operator_node = hardware_monitor.operator_node:main',
        ],
    },
)
