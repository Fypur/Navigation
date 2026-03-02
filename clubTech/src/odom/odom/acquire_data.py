"""
Ce code a servi lors de développement de l'EKF, pour vérifier la cohérence des données d'angle de l'EKF par rapport à celles de l'IMU.
Il permet d'enregistrer les données IMU et Odometry pendant 15 secondes dans des fichiers CSV, pour pouvoir les analyser par la suite.
Il s'abonne aux topics /data_imu et /odometry/filtered, et écrit les timestamps, orientations (quaternions) et positions dans des fichiers séparés.
"""


import rclpy
from rclpy.node import Node
import csv
import time
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder_15s')
        
        # Fichiers de sortie
        self.imu_file = open('imu_data.csv', 'w')
        self.odom_file = open('odom_data.csv', 'w')
        
        self.imu_writer = csv.writer(self.imu_file)
        self.odom_writer = csv.writer(self.odom_file)
        
        # En-têtes
        self.imu_writer.writerow(['timestamp', 'q_x', 'q_y', 'q_z', 'q_w'])
        self.odom_writer.writerow(['timestamp', 'pos_x', 'pos_y', 'pos_z', 'q_x', 'q_y', 'q_z', 'q_w'])
        
        # Souscriptions (QoS depth = 10 est le standard)
        self.create_subscription(Imu, '/data_imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        print("Nœud initialisé. En attente de données...")

    def get_time_sec(self, stamp):
        # Conversion du timestamp ROS 2 (sec + nanosec) en secondes float
        return stamp.sec + stamp.nanosec * 1e-9

    def imu_callback(self, msg):
        t = self.get_time_sec(msg.header.stamp)
        q = msg.orientation
        self.imu_writer.writerow([t, q.x, q.y, q.z, q.w])

    def odom_callback(self, msg):
        t = self.get_time_sec(msg.header.stamp)
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_writer.writerow([t, pos.x, pos.y, pos.z, q.x, q.y, q.z, q.w])

    def close_files(self):
        self.imu_file.close()
        self.odom_file.close()
        print("Fichiers CSV fermés avec succès.")

def main(args=None):
    rclpy.init(args=args)
    recorder = DataRecorder()
    
    start_time = time.time()
    duration = 15.0 # secondes
    
    print(f"Début de l'enregistrement pour {duration} secondes...")
    
    try:
        # Boucle principale : on "spin" (traite les callbacks) tant que le temps n'est pas écoulé
        while rclpy.ok() and (time.time() - start_time) < duration:
            rclpy.spin_once(recorder, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        recorder.close_files()
        recorder.destroy_node()
        rclpy.shutdown()
        print("Terminé.")

if __name__ == '__main__':
    main()