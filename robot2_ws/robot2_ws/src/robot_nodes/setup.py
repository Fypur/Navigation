from setuptools import setup
import os
from glob import glob

package_name = 'robot_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # 1. Enregistre le package dans l'index d'ament pour que 'ros2 run/launch' puissent le trouver
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),

        # 2. Installe le fichier package.xml dans le dossier partagé du package
        ('share/' + package_name, ['package.xml']),

        # 3. Installe tous les fichiers du dossier 'launch' (fichiers .py, .xml)
        # Le glob permet de détecter automatiquement tous les fichiers de lancement
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'lidar = robot_nodes.lidar:main',
            'detect = robot_nodes.detect:main',
            'console = robot_nodes.console:main',
            'control = robot_nodes.control:main',
            'health = robot_nodes.health:main',
            'driver = robot_nodes.driver:main',
            'automatic = robot_nodes.automatic:main',
        ],
    },
)

