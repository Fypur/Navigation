"""
 Noeud Lidar - RPLidar C1
 
 Ce noeud fait le pont entre le driver officiel  Slamtec (rplidar-ros) et le reste
 de l'architecture ROS du robot.
 
 Pré-requis :
 Le driver C1 doit tourner en parallèle :
  ros2 run rplidar_ros rplidar_node \
      --ros-args 
      -p serial_port :=/dev/ttyUSB0
      -p serial_baudrate:=460800
      -p scan_mode := DenseBoost
      -p frame_id:=laser

Ou via le launch file robot_launch.py

Paramètres ROS déclarés (modifiables) :
    - min_distance (float, defaut 0.05m) : distance min valide
    - max_distance (float, defaut 6.0m) : distance max valide
    - obstacle_range (float, defaut 0.5m) : seuil de publication /robot/lidar_obstacles             
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from msgs.msgs import Lidar

# Peut-être pas très utile mais on définit la QoS adaptée aux capteurs : best-effort, depth = 1
SENSOR_QOS = QoSProfile(
    reliability = QoSReliabilityPolicy.BEST_EFFORT,
    history = QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

class LidarNode(Node):
    
    def __init__(self):
        super().__init__('lidar')
        
        # -- Paramètres déclarés --
        self.declare_parameter('min_distance', 0.05) # en dessous le bruit est trop proche
        self.declare_parameter('max_distance', 6.0) # portée du Lidar dans notre cas pas necessairement tres grande
        self.declare_parameter('obstacle_range', 0.5) # seuil de detection
        
        self.min_dist = self.get_parameter('min_distance').value
        self.max_dist = self.get_parameter('max_distance').value
        self.obs_range = self.get_parameter('obstacle_range').value
        
        # -- Abonnnement au driver officiel
        # -- Le driver rplidar_ros publie sur le /scan en sensor_msgs/LaserScan
        
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            SENSOR_QOS,
        ) 
        
        # -- Publication --
        # -- Topic principal : tous les points valides (format utilisé pour notre robot)
        
        self.pub_lidar = self.create_publisher(Lidar, '/robot/lidar', 10)
        
        # Utilisé directement par l'algorithme de plannification et pour la détection d'obstacles
        self.pub_obstacles = self.create_publisher(Lidar, '/robot/lidar_obstacles', 10)
        
        self.get_logger().info(
            f"Noeud Lidar démarré"
            f"min={self.min_dist}m max={self.max_dist}m seuil_obstacle={self.obs_range}m"
        )
        
        # -- Callback principal --
        
        def scan_callback(self, scan: LaserScan):
            """
            Reçoit un LaserScan complet du driver C1 et publie deux messages Lidar:
                - /robot/lidar : tous les points valides
                - /robot/lidar_obstacles : points à moins de obstacle_range metres
            """
            angles_all = []
            dists_all = []
            angles_obs = []
            dists_obs = []
            
            angle = scan.angle_min
            
            for r in scan.ranges:
                # On filtre les valeurs invalides
                if math.isfinite(r) and self.min_dist <= r <= self.max_dist:
                    # on garde les angles en radians ici
                    angles_all.append(angle)
                    dists_all.append(r)
                    
                    if r <= self.obs_range:
                        angles_obs.append(angle)
                        dists_obs.append(r)
                
                angle += scan.angle_increment
            
            
            # Publication sur topic principal
            msg_all = Lidar()
            msg_all.angles = angles_all
            msg_all.distances = dists_all
            self.pub_lidar.publish(msg_all)
            
            # Publication topic obstacles (uniquement si des obstacles proches existen)
            if angles_obs:
                msg_obs = Lidar()
                msg_obs.angles = angles_obs
                msg_obs.distances = dists_obs
                self.pub_obstacles.publish(msg_obs)
            
            self.get_logger().debug(
                f"Scan reçu : {len(dists_all)} pts valides, "
                f"{len(dists_obs)} obstacles (<{self.obs_range}m)"
            )
            

# Point d'entrée

def main():
    rclpy.init()
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()