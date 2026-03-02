from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launcher import nodesToLaunch


def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        'team',
        default_value='',
        description='Namespace for all nodes'
    )
    
    namespace = LaunchConfiguration('team')
    return LaunchDescription([ns_arg] + [ Node(package=node, executable=executable, name=node, namespace=namespace) for node, executable, _ in nodesToLaunch ])
