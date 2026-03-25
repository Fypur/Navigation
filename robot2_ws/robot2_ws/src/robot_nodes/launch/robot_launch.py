from launch import LaunchDescription
from launch_ros.actions import Node


def launch_node_with_terminal(package_name: str, node_name: str):
    return Node(package=package_name,
                executable=node_name,
                name=node_name,
                output='screen',
                prefix=['xterm -e '],
                emulate_tty=True)


def generate_launch_description():

    # Liste des noms des scripts python à lancer
    robot_nodes_to_launch = ['console', 'control']
    #simulation_nodes_to_launch = ["driver"]

    ld = LaunchDescription()

    # Ouverture des différents terminaux de type xterm
    for node_name in robot_nodes_to_launch:
        ld.add_action(launch_node_with_terminal("robot_nodes", node_name))

    #for node_name in simulation_nodes_to_launch:
    #    ld.add_action(launch_node_with_terminal("simulation", node_name))

    return ld
