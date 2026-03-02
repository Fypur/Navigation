import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class SensorBridgeNode(Node):
    """
    Cette node permet de faire le pont entre les nodes des capteurs bruts (IMU et roue odométrique) et la node de l'EKF.
    Elle remappe les axes de l'IMU pour qu'ils correspondent à ceux du robot, et publie l'odométrie avec une covariance corrigée.
    Pour la roue odométrique, on publie la vitesse linéaire en x, avec une covariance faible sur x et y, et élevée sur les autres axes.
    Imposer une vitesse nulle sur y aide l'EKF à comprendre que le robot ne glisse pas latéralement.
    """
    
    
    def __init__(self):
        super().__init__('sensor_bridge_node')
        
        # Configuration des covariances
        self.odom_cov = 0.001 # faible pour les valeurs connues
        self.large_cov = 999.0 # élevée pour les valeurs inconnues

        # Initialisation des publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom/wheel', 10)
        
        # Initialisation des subscribers
        self.imu_sub = self.create_subscription(Imu, '/data_imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(TwistStamped, '/data_odo', self.odom_callback, 10)

        self.get_logger().info(f"Sensor Bridge Node properly initialized. Now publishing data on /imu/data and /odom/wheel.")

    def imu_callback(self, msg):
        """
        Fonction exécutée lorsque l'on reçoit un message de l'IMU.
        On remappe les axes pour qu'ils correspondent à ceux du robot, et on republie le message corrigé.
        """
        
        # Copie des valeurs brutes
        raw_x = msg.angular_velocity.x
        raw_y = msg.angular_velocity.y
        raw_z = msg.angular_velocity.z
        
        # Remapping : on aligne les axes du capteur sur les axes du robot
        msg.angular_velocity.z = -raw_x # Robot Z = - Capteur X  
        msg.angular_velocity.x = raw_z # Robot X = Capteur Z
        msg.angular_velocity.y = raw_y # Robot Y = Capteur Y
        
        # Pas besoin de remapper les quaternions : on n'utilise que les données du gyroscope dans l'EKF

        # Timestand et frame (on triche un peu ici)
        msg.header.frame_id = "base_link" # ici, on triche !
        # On dit que le message vient DIRECTEMENT du robot (base_link), et pas de l'imu_link.
        # Vous pouvez vous branchez si vous voulez, mais armez-vous de patience si vous voulez faire quelque chose de propre !        
            
        self.imu_pub.publish(msg)

    def odom_callback(self, msg):
        """
        Fonction exécutée lorsque l'on reçoit un message de la roue odométrique.
        On publie un message d'odométrie avec une covariance corrigée, en imposant une vitesse nulle selon y pour 
        imposer le comportement non-holonomique (en gros, le robot ne glisse pas latéralement).
        """
        
        # Récupération de la donnée
        current_vel_x = msg.twist.linear.x
        
        # Création du message d'odométrie et remplissage du header
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Remplissage des vitesses
        odom_msg.twist.twist.linear.x = current_vel_x
        odom_msg.twist.twist.linear.y = 0.0 
        
        # Matrice de covariance : on ne remplit que la diagonale 
        covariance = [0.0] * 36
        covariance[0] = 0.001 # Vx : très précis
        covariance[7] = 0.001 # Vy : très précis (pour imposer 0)
        
        # Pour les autres axes, on ne sait pas, donc incertitude max pour que l'EKF ignore ces valeurs
        covariance[14] = 999.0  # Vz
        covariance[21] = 999.0  # Vroll
        covariance[28] = 999.0  # Vpitch
        covariance[35] = 999.0  # Vyaw (Vitesse de rotation)

        odom_msg.twist.covariance = covariance
        
        # Publication du message
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()