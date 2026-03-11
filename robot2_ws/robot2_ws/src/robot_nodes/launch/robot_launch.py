from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Liste des noms des scripts python à lancer
    nodes_to_launch = ['automatic', 'console', 'control', 'detect', 'health', 'lidar']

    ld = LaunchDescription()

    # Ouverture des différents terminaux de type xterm
    for node_name in nodes_to_launch:
        node_action = Node(package='robot_nodes',
                           executable=node_name,
                           name=node_name,
                           output='screen',
                           prefix=['gnome-terminal -- '],
                           emulate_tty=True)

        ld.add_action(node_action)

    return ld
