import rclpy
from math import atan2, sin, cos
from rclpy.node import Node
from nav_msgs.msg import Odometry # Message renvoyé par l'EKF
from interfaces.msg import RobotState # Message utilisé pour communiquer avec les autres nodes du code
from tf_transformations import euler_from_quaternion
from odom.utils import *


class OffsetNode(Node):
    """
    Cette node permet de traiter les informations en sortie du filtre de Kalman étendu.
    Elle ajoute un offset à la position du robot pour tenir compte de sa position initiale et reformate les messages pour que 
    ceux-ci soient compatibles avec les autres node du code. Enfin, elle facilite l'implémentation du recalage sur les murs pour éviter
    la dérive.
    """
    
    
    def __init__(self):
        super().__init__('offset_node')
        
        # Etat du robot
        self.offset_position = initial_offset
        self.offset_position["cos"] = cos(self.offset_position["yaw"])
        self.offset_position["sin"] = sin(self.offset_position["yaw"])

        # Création des subscribers
        self.odom_subscriber = self.create_subscription(Odometry, ekf_output_topic_name, self.odom_callback, 10)
        
        # Création des publishers
        self.odom_pub = self.create_publisher(RobotState, offset_output_topic_name, 10) # TODO : changer le type de message
        
        # TODO : ajouter un service pour gérer le recalage sur les murs

        self.get_logger().info(f"Offset Node properly initialized.")

    def normalize_angle(self, angle):
        """
        Cette méthode normalise l'angle pour qu'il reste dans l'intervalle [-pi, pi].
        """
        return atan2(sin(angle), cos(angle))

    def odom_callback(self, message):
        # Récupération de la position
        x_ekf = message.pose.pose.position.x
        y_ekf = message.pose.pose.position.y

        # Récupération de l'orientation et conversion en angles d'Euler
        orientation = message.pose.pose.orientation

        quaternion_list = [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ]

        (_, _, yaw_ekf) = euler_from_quaternion(quaternion_list)        

        # Récupération de la vitesse linéaire et angulaire
        speed = message.twist.twist.linear.x    
        angular_speed = message.twist.twist.angular.z

        # Récupération de l'offset
        x_offset = self.offset_position['x']
        y_offset = self.offset_position['y']
        yaw_offset = self.offset_position['yaw']

        # Application de l'offset
        cos_yaw = self.offset_position["cos"]
        sin_yaw = self.offset_position["sin"]

        x = x_offset + (x_ekf * cos_yaw - y_ekf * sin_yaw)
        y = y_offset + (x_ekf * sin_yaw + y_ekf * cos_yaw)

        # Calcul et normalisation de l'angle
        yaw = self.normalize_angle(yaw_ekf + yaw_offset)

        # Création du message à publier
        odometry_message = RobotState()
        odometry_message.x = x
        odometry_message.y = y
        odometry_message.theta = yaw
        odometry_message.v = speed
        odometry_message.omega = angular_speed

        # Publication du message
        self.odom_pub.publish(odometry_message)


def main(args=None):
    rclpy.init(args=args)
    
    node = OffsetNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass 
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()