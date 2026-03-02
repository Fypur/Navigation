import os
import time
import rclpy
import numpy as np
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation
from imu.imu.utils import *

from adafruit_bno08x.uart import BNO08X_UART
from adafruit_bno08x import (
    BNO_REPORT_LINEAR_ACCELERATION, 
    BNO_REPORT_GYROSCOPE, 
    BNO_REPORT_ROTATION_VECTOR
)

class IMU_Node(Node):
    """
    Cette node lit les données de l'IMU BNO080 via UART, effectue une calibration initiale
    pour définir l'orientation de référence, puis publie les données IMU corrigées sur un topic ROS2.
    
    Note : finalement, nous n'utilisons pas les données d'orientation, seules celles du gyroscope sont utilisées.
    """
    
    
    def __init__(self):
        super().__init__("IMU_node")
        
        # Configuration
        self.PORT = PORT # Port série de l'IMU
        self.BAUD = BAUD # Vitesse de communication
        self.RESET_GPIO_CHIP = RESET_GPIO_CHIP # Puce GPIO pour le reset matériel
        self.RESET_GPIO_LINE = RESET_GPIO_LINE # Ligne GPIO pour le reset matériel
        
        # Calibration configuration
        self.calibration_samples = calibration_samples
        self.calibration_iterations_limit = calibration_iterations_limit
        self.initial_yaw_target_deg = initial_yaw 
        
        self.offset_rotation = None
        
        # Hardware initialization
        self.hardware_reset()
        self.uart = None
        self.open_port()
        
        # Connexion à l'IMU via SHTP
        self.bno = None
        handshake = self.SHTP_handshake() 
        
        # Si la connexion a réussi, on active les features nécessaires
        if handshake:
            try:
                time.sleep(0.5) 
                self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR) # Vecteur de rotation
                self.bno.enable_feature(BNO_REPORT_GYROSCOPE) # Gyroscope
                self.bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION) # Accélération linéaire
                self.get_logger().info("Features enabled.")
            except Exception as e:
                self.get_logger().warn(f"Feature enable warning: {e}")

        # Création du publisher et du timer
        self.publisher = self.create_publisher(Imu, topic, 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("IMU node initialized.")
        
    def hardware_reset(self):
        """
        Cette méthode effectue un reset matériel de l'IMU via GPIO.
        """
        try: # On essaie de faire le reset
            self.get_logger().info("Resetting hardware...")
            os.system(f"gpioset {self.RESET_GPIO_CHIP} {self.RESET_GPIO_LINE}=0")
            time.sleep(0.1)
            os.system(f"gpioset {self.RESET_GPIO_CHIP} {self.RESET_GPIO_LINE}=1")
            time.sleep(0.7)
            return True
        except Exception as e: # Si on rencontre une erreur, on affiche un message d'erreur et on retourne False
            self.get_logger().error(f"Hardware reset failed: {e}")
            return False
            
    def open_port(self):
        """
        Cette méthode ouvre le port série pour communiquer avec l'IMU.
        """
        try: # On essaie d'ouvrir le port série
            self.uart = serial.Serial(self.PORT, self.BAUD, timeout=0.1)
            self.uart.reset_input_buffer() 
            return True
        except Exception as e: # Si on rencontre une erreur, on affiche un message d'erreur et on retourne False
            self.get_logger().error(f"Serial open failed: {e}")
            return False
        
    def SHTP_handshake(self):
        """
        Cette méthode effectue la négociation SHTP pour se connecter à l'IMU.
        """
        try: # On essaie de se connecter à l'IMU via SHTP
            self.bno = BNO08X_UART(self.uart)
            return True
        except Exception as e: # Si on rencontre une erreur, on affiche un message d'erreur et on retourne False
            self.get_logger().error(f"Handshake failed: {e}")
            return False
    
    def calibrate(self):
        """
        Cette méthode effectue la calibration initiale de l'IMU pour définir l'orientation de référence.
        Cette calibration consiste à lire plusieurs échantillons du vecteur de rotation,
        à calculer la moyenne, puis à déterminer l'offset nécessaire pour aligner l'orientation
        mesurée avec l'angle de yaw initial souhaité.
        """
        self.get_logger().info(f"Starting calibration ({self.calibration_samples} samples)...")
        
        # Collecte des échantillons
        current_samples = 0
        quaternion_sum = np.zeros(4)
        iterations = 0
        
        while current_samples < self.calibration_samples:
            iterations += 1
            if iterations > self.calibration_iterations_limit:
                self.get_logger().warn("Calibration Timeout!")
                return False

            try: # Lecture du quaternion brut
                raw_quat = self.bno.quaternion
                
                if raw_quat is not None and np.linalg.norm(raw_quat) > 0.01:
                    
                    # 1. On convertit en Rotation objet
                    r_raw = Rotation.from_quat(raw_quat)
                    
                    # 3. On somme le quaternion (remis en tableau numpy)
                    quaternion_sum += r_raw.as_quat()
                    
                    current_samples += 1
                    
                    if current_samples % 10 == 0:
                        self.get_logger().info(f"Calibrating: {current_samples}/{self.calibration_samples}")
                
                time.sleep(0.01)

            except Exception as e:
                self.get_logger().warn(f"Calib read error: {e}")
        
        # Moyenne du résultat
        avg_quat = quaternion_sum / self.calibration_samples
        measured_rotation_corrected = Rotation.from_quat(avg_quat)
        
        # Calcul de la rotation cible
        target_rotation = Rotation.from_euler('z', self.initial_yaw_target_deg, degrees=True)
        
        # Calcul de l'offset de rotation
        self.offset_rotation = target_rotation * measured_rotation_corrected.inv()
        
        self.get_logger().info(f"Calibration done! Yaw set to {self.initial_yaw_target_deg} degrees.")
        return True
            
        
    def timer_callback(self):
        """
        Méthode appelée périodiquement par le timer pour lire les données de l'IMU,
        appliquer la calibration si nécessaire, et publier les données corrigées.
        """
        try:
            if not self.bno: return

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"

            # Lecture du quaternion brut
            raw_quat = self.bno.quaternion 
            
            if raw_quat is not None and np.linalg.norm(raw_quat) > 0.01: # On vérifie que la lecture est valide
                
                # Calibration si nécessaire
                if self.offset_rotation is None:
                    if not self.calibrate():
                        return 

                # Lecture du quaternion brut
                r_raw = Rotation.from_quat(raw_quat)
                
                # 3. Application du Tare (calibration)
                final_rotation = self.offset_rotation * r_raw
                final_quat = final_rotation.as_quat()
                
                msg.orientation.x = final_quat[0]
                msg.orientation.y = final_quat[1]
                msg.orientation.z = final_quat[2]
                msg.orientation.w = final_quat[3]
                msg.orientation_covariance = [  0.0001, 0.0,    0.0,    # Variance Roll (X)
                                                0.0,    0.0001, 0.0,    # Variance Pitch (Y)
                                                0.0,    0.0,    0.001]  # Variance Yaw (Z)
            
            # Lecture du gyroscope
            gyro = self.bno.gyro
            if gyro:
                msg.angular_velocity.x = gyro[0]
                msg.angular_velocity.y = gyro[1]
                msg.angular_velocity.z = gyro[2]
                msg.angular_velocity_covariance = [ 0.00001, 0.0,     0.0,
                                                    0.0,     0.00001, 0.0,
                                                    0.0,     0.0,     0.00001]

            # Lecture de l'accélération linéaire
            lin_accel = self.bno.linear_acceleration
            if lin_accel:
                msg.linear_acceleration.x = lin_accel[0]
                msg.linear_acceleration.y = lin_accel[1]
                msg.linear_acceleration.z = lin_accel[2]
                msg.linear_acceleration_covariance = [  0.001, 0.0,   0.0,
                                                        0.0,   0.001, 0.0,
                                                        0.0,   0.0,   0.001]

            # Publication du message
            self.publisher.publish(msg)
                
        except Exception as e:
            self.get_logger().debug(f"IMU Read Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMU_Node()
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()