import rclpy
import random
import time
from rclpy.node import Node
from robot_msgs.msg import Health, Lidar


class LidarNode(Node):

    def __init__(self):
        super().__init__("lidar")

        self.pub_health = self.create_publisher(Health, "/robot/health", 10)
        self.pub = self.create_publisher(Lidar, "/robot/sensor/LiDAR/status", 10)

        self.create_timer(0.1, self.heartbeat)
        self.create_timer(0.2, self.scan)

        self.start = time.time()

        self.get_logger().info("Lidar node launched")

    def generate_noise(self):
        pts = []
        for i in range(120):
            angle = i * 3
            dist = random.uniform(200, 400)
            pts.append([angle, dist])
        return pts

    def generate_object(self):
        pts = []
        center = random.uniform(-30, 30)
        base = random.uniform(0, 50)
        for i in range(-8, 9):
            angle = center + i
            if angle < 0:
                angle += 360
            dist = base + random.uniform(-1, 1)
            pts.append([angle, dist])
            self.get_logger().info(f"Insert Object {angle},{dist}")
        return pts

    def heartbeat(self):
        h = Health(state="Hello", name="Lidar")
        self.pub_health.publish(h)

    def scan(self):
        msg = Lidar()
        t = time.time() - self.start
        object_present = int(t) % 9 == 0

        pts = self.generate_noise()
        if object_present:
            pts += self.generate_object()

        for angle, dist in pts:
            msg.distances.append(dist)
            msg.angles.append(angle)

        # for a in range(360):
        #    msg.angles.append(float(a))
        #    if object_present and abs(a)<5:
        #        msg.distances.append(30.0)
        #    else:
        #        msg.distances.append(random.uniform(80,200))

        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(LidarNode())
    rclpy.shutdown()
