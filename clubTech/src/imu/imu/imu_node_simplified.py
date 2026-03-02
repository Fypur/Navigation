# TODO : importer ce code sur la RASPI et tester avec le filtre de Kalman étendu

import os
import time
import rclpy
import numpy as np
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation
from imu.utils import *

from adafruit_bno08x.uart import BNO08X_UART
from adafruit_bno08x import (
    BNO_REPORT_LINEAR_ACCELERATION, 
    BNO_REPORT_GYROSCOPE, 
    BNO_REPORT_ROTATION_VECTOR
)

class IMU_Node(Node):
    """
    Cette node lit les données de l'IMU BNO080 via UART et publie les données du gyroscope sur le topic ROS2 spécifié dans le fichier utils.
    Il s'agit d'une version simplifiée de la node imu_node_all_data.py, sans calibration initiale ni utilisation des données d'orientation.
    Cette version permet une initialisation plus rapide de l'EKF en s'affranchissant de la calibration. 
    """
    
    
    def __init__(self):
        super().__init__("IMU_node")
        
        # Configuration
        self.PORT = PORT # Port série de l'IMU
        self.BAUD = BAUD # Vitesse de communication
        self.RESET_GPIO_CHIP = RESET_GPIO_CHIP # Puce GPIO pour le reset matériel
        self.RESET_GPIO_LINE = RESET_GPIO_LINE # Ligne GPIO pour le reset matériel
        
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
                self.bno.enable_feature(BNO_REPORT_GYROSCOPE) # Gyroscope
                self.get_logger().info("Features enabled.")
            except Exception as e:
                self.get_logger().warn(f"Feature enable warning: {e}")

        # Création du publisher et du timer
        self.publisher = self.create_publisher(Imu, topic, 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("IMU node initialized. Now publishing data...")
        
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

            # Lecture du gyroscope
            gyro = self.bno.gyro
            if gyro:
                msg.angular_velocity.x = gyro[0]
                msg.angular_velocity.y = gyro[1]
                msg.angular_velocity.z = gyro[2]
                msg.angular_velocity_covariance = [ 0.00001, 0.0,     0.0,
                                                    0.0,     0.00001, 0.0,
                                                    0.0,     0.0,     0.00001]
                msg.header.stamp = self.get_clock().now().to_msg()

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