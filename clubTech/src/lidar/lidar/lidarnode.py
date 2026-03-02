from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D

from lidar.filter import LidarFilter
import copy

RViz = True

class Lidar(Node):
    def __init__(self):
        super().__init__('lidar')

        self.lidar_filter = LidarFilter()
        self.last_pos = None

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        self.create_subscription(Odometry, '/pos', self.pos_callback, 1)

        self.pub_opp = self.create_publisher(Pose2D, '/lidar_opp', 1)
        if RViz:
            self.pub_RViz = self.create_publisher(LaserScan, '/scan_filtered', 1)

    def scan_callback(self, scan_msg: LaserScan):
        self.last_pos = (0,0,0,0,0,0,0)
        if self.last_pos is None:
            self.get_logger().warn("Pas encore de pose reçue sur /pos, je ne filtre pas.")
            return
        
        filtered_ranges, filtered_coordinates = self.lidar_filter.filter(scan_msg, self.last_pos)
        filtered_scan_msg = copy.deepcopy(scan_msg)
        filtered_scan_msg.ranges = filtered_ranges

        opp_pos = self.lidar_filter.opp_pos(filtered_coordinates)
        pose = Pose2D()
        pose.x = opp_pos[0]
        pose.y = opp_pos[1]
        pose.theta = 0.0
        self.pub_opp.publish(pose)

        if RViz:
            self.pub_RViz.publish(filtered_scan_msg)
            self.get_logger().info(f"Publié /data_lidar ({len(filtered_ranges)} ranges)")

    def pos_callback(self, pos_msg: Odometry):
        p = pos_msg.pose.pose.position
        o = pos_msg.pose.pose.orientation
        self.last_pos = (p.x, p.y, p.z, o.x, o.y, o.z, o.w)
