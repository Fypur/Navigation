from glob import glob
import os
from setuptools import setup, find_packages

package_name = 'simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # Use explicit list instead of find_packages()
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'assets'),
         glob('simulation/game/assets/*')),
    ],
    install_requires=[
        'setuptools',
        'pygame',
        'std_msgs',
        'pymunk',
    ],
    zip_safe=True,
    maintainer='salade',
    maintainer_email='your_email@example.com',
    description='Robot simulation with Pygame/Pymunk',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_simulation = simulation.run_simulation:main',
        ],
    },

)