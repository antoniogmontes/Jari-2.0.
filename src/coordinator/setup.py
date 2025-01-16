from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'coordinator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antonio',
    maintainer_email='antonio@todo.todo',
    description='Paquete coordinador de los movimientos y emociones del robot, asi como de la coordinacion con un servidor web flask',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'coordinator_node=coordinator.coordinator_node:main',
        ],
    },
)
