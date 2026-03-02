"""
Ce fichier permet d'acquérir la variance de l'IMU lorsque le robot immobile.
Cette variance est sauvegardée dans un fichier YAML, ce qui nous a permis de régler les covariances dans l'EKF.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuCalibrationNode(Node):
    def __init__(self):
        super().__init__('imu_calibration_node')

        # Paramètres
        self.declare_parameter('topic_name', '/data_imu')
        self.declare_parameter('duration_sec', 60.0)
        self.declare_parameter('output_file', 'imu_params.yaml')

        topic = self.get_parameter('topic_name').get_parameter_value().string_value
        self.duration = self.get_parameter('duration_sec').get_parameter_value().double_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value

        # Stockage des données
        self.linear_acc_x = []
        self.linear_acc_y = []
        self.linear_acc_z = []
        self.angular_vel_x = []
        self.angular_vel_y = []
        self.angular_vel_z = []

        self.start_time = None
        self.collecting = True

        # Subscriber
        self.subscription = self.create_subscription(
            Imu,
            topic,
            self.listener_callback,
            10
        )
        
        self.get_logger().info(f"Début de la calibration sur {topic} pour {self.duration} secondes...")
        self.get_logger().info("Gardez l'IMU parfaitement immobile !")

    def listener_callback(self, msg):
        if not self.collecting:
            return

        # Initialiser le temps au premier message reçu
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        # Vérifier si la durée est écoulée
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9

        if elapsed >= self.duration:
            self.collecting = False
            self.calculate_and_save()
            return

        # Enregistrement des données brutes
        self.linear_acc_x.append(msg.linear_acceleration.x)
        self.linear_acc_y.append(msg.linear_acceleration.y)
        self.linear_acc_z.append(msg.linear_acceleration.z)

        self.angular_vel_x.append(msg.angular_velocity.x)
        self.angular_vel_y.append(msg.angular_velocity.y)
        self.angular_vel_z.append(msg.angular_velocity.z)
        
        # Feedback visuel toutes les 5 secondes (optionnel)
        if int(elapsed) % 5 == 0 and int(elapsed * 10) % 10 == 0:
             self.get_logger().info(f"Calibration en cours... {int(elapsed)}s / {int(self.duration)}s")

    def calculate_and_save(self):
        self.get_logger().info("Calcul des variances en cours...")

        if len(self.linear_acc_x) == 0:
            self.get_logger().error("Aucune donnée reçue ! Vérifiez le topic.")
            rclpy.shutdown()
            return

        # Calcul des variances avec Numpy
        # Note : On calcule la variance (sigma^2)
        var_acc_x = np.var(self.linear_acc_x)
        var_acc_y = np.var(self.linear_acc_y)
        var_acc_z = np.var(self.linear_acc_z)

        var_gyro_x = np.var(self.angular_vel_x)
        var_gyro_y = np.var(self.angular_vel_y)
        var_gyro_z = np.var(self.angular_vel_z)

        # Création du contenu du fichier YAML
        yaml_content = f"""# Calibration IMU BNO086
# Généré automatiquement
# Nombre d'échantillons: {len(self.linear_acc_x)}

imu_config:
  ros__parameters:
    # Variances (sigma^2) calculées
    acc_variance_x: {var_acc_x:.8f}
    acc_variance_y: {var_acc_y:.8f}
    acc_variance_z: {var_acc_z:.8f}
    
    gyro_variance_x: {var_gyro_x:.8f}
    gyro_variance_y: {var_gyro_y:.8f}
    gyro_variance_z: {var_gyro_z:.8f}
"""
        
        # Sauvegarde fichier
        with open(self.output_file, 'w') as f:
            f.write(yaml_content)

        self.get_logger().info(f"Succès ! Les variances ont été sauvegardées dans '{self.output_file}'")
        
        # Arrêt propre de la node
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImuCalibrationNode()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()