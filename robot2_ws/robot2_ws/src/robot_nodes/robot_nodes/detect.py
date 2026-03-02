import rclpy
import math
from rclpy.node import Node
from robot_msgs.msg import Health,Lidar,Detect,Command

class DetectNode(Node):

    def __init__(self):
        super().__init__("detect")

        self.threshold=50.0
        self.counter = 0

        self.pub_health=self.create_publisher(Health,"/robot/health",10)
        self.pub=self.create_publisher(Detect,"/robot/detect/status",10)

        self.create_subscription(Command,"/robot/detect/command",self.config_cb,10)
        self.create_subscription(Lidar,"/robot/sensor/LiDAR/status",self.lidar_cb,10)

        self.create_timer(0.1,self.heartbeat)

    def polar_to_cartesian(iself, angle, distance):
        rad = math.radians(angle)
        x = distance * math.cos(rad)
        y = distance * math.sin(rad)
        return [x, y]

    def detect_object(self, points):
        # convertir en XY
        pts = [self.polar_to_cartesian(a, d) for a, d in points]

        clusters = []

        for p in pts:
            added = False

            for cluster in clusters:
                cx = sum(x for x,_ in cluster)/len(cluster)
                cy = sum(y for _,y in cluster)/len(cluster)

                dist = math.sqrt((p[0]-cx)**2 + (p[1]-cy)**2)

                if dist < 25:  # rayon objet
                    cluster.append(p)
                    added = True
                    break

            if not added:
                clusters.append([p])

        # garder gros clusters
        clusters = [c for c in clusters if len(c) > 8]

        if not clusters:
            return None

        obj = max(clusters, key=len)



        distance = math.sqrt(cx**2 + cy**2)
        angle = math.degrees(math.atan2(cy,cx))

        return [angle, distance]

    def in_front_cone(self, angle):
        return angle >= 350 or angle <= 10

    def heartbeat(self):
        self.pub_health.publish(Health(state="Hello",name="detect"))

    def config_cb(self,msg):
        if msg.action=="config":
            self.threshold=msg.distance

    def lidar_cb(self,msg):
        #self.counter +=1
        #if self.counter % 20 == 0:
        #        self.pub.publish(Detect(action="objet détecté",distance=self.threshold))

        ##for d in msg.distances:
        #    if d<self.threshold:
        #        self.pub.publish(Detect(action="objet détecté",distance=d))
        #        return

        pts =[]
        for angle, dist in zip(msg.angles,msg.distances):
            if self.in_front_cone(angle):
               pts.append([angle,dist])
        result=self.detect_object(pts)
        if result and result[1] < self.threshold:
            self.pub.publish(Detect(action="objet détecté",distance=result[1]))


def main():
    rclpy.init()
    rclpy.spin(DetectNode())
    rclpy.shutdown()

