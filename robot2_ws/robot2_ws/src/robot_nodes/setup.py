from setuptools import setup

package_name = 'robot_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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

