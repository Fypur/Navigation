# TODO : ajuster les délais des TimerAction si on rencontre des problèmes au lancement

import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    """
    Lance les nodes nécessaires pour l'odométrie du robot : IMU, roue odométrique, sensor bridge, et EKF.
    """
    
    # Récupérer le chemin vers votre package pour trouver le fichier YAML
    pkg_odom_share = get_package_share_directory('odom')
    
    # Chemin complet vers le fichier de configuration du filtre de Kalman
    ekf_config_path = os.path.join(pkg_odom_share, 'config', 'ekf.yaml')

    # Node IMU
    imu_node = Node(
        package='imu',
        executable='imu_node',
        name='imu_node',
        output='screen'
    )

    # Node Roue Odométrique    
    odo_node = Node(
        package='odo',
        executable='roue_odo',
        name='odo_node',
        output='screen'
    )
    
    # Node Sensor Bridge
    sensor_bridge_node = Node(
        package='odom',
        executable='sensor_bridge',
        name='sensor_bridge_node',
        output='screen'
    )

    # Node EKF 
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )
    
    # Node Offset
    offset_node = Node(
        package='odom',
        executable='offset_node',
        name='offset_node',
        output='screen'
    ) 

    # Les TimerAction permettent de définir un délai entre le lancement des nodes.
    # Si on rencontre des problèmes au lancement, essayez d'augmenter les délais pour laisser plus de temps aux capteurs pour se connecter.
    return LaunchDescription([
        imu_node,
        TimerAction(
            period=8.0,
            actions=[odo_node]
        ),
        TimerAction(
            period= 10.0,
            actions=[sensor_bridge_node]
        ),
        TimerAction(
            period = 12.0,
            actions = [ekf_node]
        ),
        TimerAction(
            period = 14.0,
            actions = [offset_node]
        )
        
    ])