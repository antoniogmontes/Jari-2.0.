from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'micro'

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
    maintainer_email='antonio.gmontes@alumnos.upm.es',
    description='Paquete de prueba simulacion del funcionamiento del microfono + altavoz',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'micro_node=micro.micro_node:main',
        ],
    },
)
