import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from odo.can_interface import CANInterface
from numpy import pi
from collections import deque

class OdometricWheel(Node):
    """
    Cette node permet de récupérer les données de la roue odométrique, et les transmet au sensor_bridge.
    Elle applique un filtre de moyenne glissante pour lisser les données de vitesse avant de les publier. 
    Cela lisse les valeurs de vitesse tout en maintenant une réactivité suffisante pour l'EKF.
    """
    def __init__(self):
        super().__init__('odo_node')
        
        # Configuration
        self.topic_name = '/data_odo'
        self.publisher_ = self.create_publisher(TwistStamped, self.topic_name, 10)
        
        # Paramètres physiques
        self.tick_to_meter = 1 / 2048 * (pi * 3.4e-2) / 0.9
        self.deadband = 0.01 
        
        # Configuration du buffer pour la moyenne glissante
        self.window_size = 10
        self.speed_buffer = deque(maxlen=self.window_size)
        
        # Timer
        self.freq = 50.0
        self.timer = self.create_timer(1.0 / self.freq, self.timer_callback)
        
        # Variables d'état
        self.previous_time = self.get_clock().now().nanoseconds / 1e9
        self.previous_pos = None
        
        # Interface CAN
        self.can = CANInterface(channel='can1') 
        
        self.get_logger().info("Roue odométrique initialisée.")
    
    def get_smoothed_speed(self, raw_speed):
        """
        Ajoute la vitesse brute au buffer et retourne la moyenne.
        """
        self.speed_buffer.append(raw_speed)
        return sum(self.speed_buffer) / len(self.speed_buffer)

    def timer_callback(self):
        """
        Fonction exécutée à chaque tick du timer. Elle lit les ticks de la roue, calcule la vitesse, applique le filtre, et publie le résultat.
        """
        new_ticks = self.can.get_last_msg()
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Gestion d'une lecture vide ou d'une absence de message CAN
        if new_ticks is None:
            if self.previous_pos is None:
                self.get_logger().warn("Attente CAN...", throttle_duration_sec=2.0)
                return
            else:
                # Si on a déjà une position précédente, on considère que la roue n'a pas bougé (0 ticks) pour éviter les sauts de vitesse
                current_ticks = self.previous_pos
        else:
            current_ticks = new_ticks

        # Initialisation des variables d'état lors de la première lecture valide
        if self.previous_pos is None:
            self.previous_pos = current_ticks
            self.previous_time = current_time
            return

        dt = current_time - self.previous_time
        
        # Sécurité pour éviter une division par zéro
        if dt < 0.001: 
            return
        
        # Calcul de la vitesse instantanée
        d_ticks = current_ticks - self.previous_pos
        raw_speed = (d_ticks * self.tick_to_meter) / dt
        
        # Mise à jour des états pour le prochain tour
        # IMPORTANT : On met à jour même si d_ticks est 0 pour avancer le temps
        self.previous_pos = current_ticks 
        self.previous_time = current_time

        # Application de la deadband pour éviter les petites fluctuations de vitesse
        if abs(raw_speed) < self.deadband:
            raw_input = 0.0
        else:
            raw_input = raw_speed

        # Application du filtre de moyenne glissante
        smoothed_speed = self.get_smoothed_speed(raw_input)
        
        # Publication des données
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(smoothed_speed)
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometricWheel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.can.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    