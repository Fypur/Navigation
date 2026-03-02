from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- AJOUTEZ CES LIGNES POUR INSTALLER CONFIG ET LAUNCH ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tom Jeandé',
    maintainer_email='tom.jeande@student-cs.fr',
    description='Fusion odométrie roue centrale et IMU BNO086',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_bridge = odom.sensor_bridge:main',
            'acquire_data = odom.acquire_data:main',
            'offset_node = odom.offset_node:main',
        ],
    },
)