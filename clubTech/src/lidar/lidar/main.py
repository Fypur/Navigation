import rclpy
from lidar.lidarnode import Lidar

def main(args=None):
    rclpy.init(args=args)
    lidar_node = Lidar()
    rclpy.spin(lidar_node)
    lidar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
