import rclpy
from rclpy.node import Node
from msgs.msg import Lidar, RPMs
from geometry_msgs.msg import Pose2D
import math
import numpy as np
import time
from numpy.typing import NDArray

WHEEL_RADIUS = 0.04
LX = 0.15
LY = 0.15
GEAR_RATIO = 1.0


class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization')

        # -- Etat interne de la position --
        self.position = np.array([0., 0.])
        self.prev_position = np.array([0., 0.])
        self.theta = 0.0
        self.prev_theta = 0.0
        self.prev_lidar_points = None
        self.lidar_points = None

        self.last_time = time.time()

        # -- Abonnements aux topics --

        # Abonnement au Lidar pour la correction (Basse fréquence)
        self.create_subscription (Lidar, '/robot/lidar', self.lidar_callback, 1)

        # -- Publication --
        self.pub_pos = self.create_publisher(Pose2D, '/robot/pos', 10)

        self.get_logger().info("Noeud Localization (Encodeurs et Lidar) démarré")
        # self.test_lidar_pos()

    # -- Callback lent : Correction Lidar --
    def lidar_callback(self, msg: Lidar):
        """
        Recalage de la position via l'environnement pour annuler la dérive
        des encodeurs.
        """

        if self.prev_lidar_points is None:
            self.prev_lidar_points = self.get_points(msg)
            return

        self.lidar_points = self.get_points(msg)

        rotation, translation = self.icp(self.lidar_points, self.prev_lidar_points)

        rot_matrix = np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]])

        self.position = rot_matrix @ translation + self.prev_position

        delta_theta = np.arctan2(rotation[1, 0], rotation[0, 0])
        self.theta = self._angle_wrap(self.prev_theta + delta_theta)

        self.prev_position = self.position.copy()
        self.prev_theta = self.theta
        self.prev_lidar_points = self.lidar_points.copy()

        self._publish_pose()

    # -- Méthodes utilitaires --

    def get_points(self, scan: Lidar):
        """Convertit un scan Lidar en nuage de points cartésiens"""
        points = []

        for angle, dist in zip(scan.angles, scan.distances):
            points.append(np.array([dist * np.cos(angle), dist * np.sin(angle)]))

        return np.array(points)

    def icp(self, source: NDArray[np.float64], target: NDArray[np.float64]):

        MAX_ITERATIONS = 50
        MATCH_DISTANCE_THRESHOLD = 0.1
        TOLERANCE = 1e-6

        def find_closest_match(point: NDArray[np.float64]):
            distances = np.linalg.norm(target - point, axis=1) # array of distances from point to every point of target
            index = distances.argmin()

            if distances[index] < MATCH_DISTANCE_THRESHOLD:
                return target[index]
            return None  # filter bad matches, since icp is vulnerable to outliers

        def get_center_of_mass(points: NDArray[np.float64]):
            return np.sum(points, axis=0) / len(points)

        def best_transformation(transformation_source: NDArray[np.float64], transformation_target: NDArray[np.float64]):
            source_centroid = get_center_of_mass(transformation_source)
            target_centroid = get_center_of_mass(transformation_target)

            centered_source = transformation_source - source_centroid
            centered_target = transformation_target - target_centroid

            covariance_matrix = centered_source.T @ centered_target


            #using svd
            #u, s, vh = np.linalg.svd(covariance_matrix) # magic optimisation bullshit
            #prevent reflections
            # d = +1 if both are rotations or both are reflections → corrects all 4 cases
            #d = np.sign(np.linalg.det(u) * np.linalg.det(vh))
            #D = np.diag([1, d])
            #rotation = vh.T @ D @ u.T

            angle = np.arctan2(
                covariance_matrix[0, 1] - covariance_matrix[1, 0],
                covariance_matrix[0, 0] + covariance_matrix[1, 1]
            )

            rotation = np.array([
                [np.cos(angle), -np.sin(angle)],
                [np.sin(angle),  np.cos(angle)]
            ])

            translation = target_centroid - rotation @ source_centroid

            return rotation, translation


        src = source.copy()
        total_rotation = np.identity(2)
        total_translation = np.zeros(2)


        for i in range(MAX_ITERATIONS):
            # for each point in src, find match in target by checking closest point
            matched_targets = []
            filtered_src = []
            for src_point in src:
                match = find_closest_match(src_point)
                if match is not None: #filter bad matches
                    filtered_src.append(src_point)
                    matched_targets.append(match)

            if len(filtered_src) < 3: # Need at least a few points to guess a transform
                return total_rotation, total_translation

            # Compute optimal transformation
            rotation, translation = best_transformation(np.array(filtered_src), np.array(matched_targets))

            # Move source to be closer to the target using optimal calculated transformation
            src = (rotation @ src.T).T + translation

            # Accumulate total transformation
            total_rotation = rotation @ total_rotation
            total_translation = rotation @ total_translation + translation

            # Check for convergence
            if np.linalg.norm(translation) < TOLERANCE and np.linalg.norm(rotation - np.identity(2)) < TOLERANCE:
                break

        return total_rotation, total_translation




    def _publish_pose(self):
        msg = Pose2D()
        msg.x = float(self.position[0])
        msg.y = float(self.position[1])
        msg.theta = 0.
        self.get_logger().info(f"Sent position x={round(msg.x, 2)}, y={round(msg.y, 2)}")
        self.pub_pos.publish(msg)

    def _angle_wrap(self, angle):
        # Ramène un angle à [-pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def test_icp(self):

        count = 0
        for i in range(100):
            points = np.random.random(size=(200, 2))
            rot = np.random.random() * 2
            translat = np.random.random(2) * 2

            angle = np.radians(rot)

            R = np.array([
                [np.cos(angle), -np.sin(angle)],
                [np.sin(angle),  np.cos(angle)]
            ])

            next = points @ R.T + translat

            rotation, translation = self.icp(points, next)

            t_error = np.linalg.norm(translation - translat)
            r_error = rot - np.degrees(np.arctan(rotation[1][0] / rotation[0][0]))

            if t_error > 0.5 or r_error > 1:
                print(f"translation gap : {t_error}")
                print(f"angle gap {r_error}")
                count += 1
        print(f"count {count}")

    def test_lidar_pos(self):

        init_scan = np.array([[3, 3], [3, 0], [3, -3]])
        #post_scan = np.array([[2, 3], [2, 0], [2, -3]])
        post_scan = np.array([[0, -2], [-2, -2], [-2, -2]])
        init_rot = np.radians(0)

        rotation, translation = self.icp(post_scan, init_scan)

        rot_matrix = np.array([[np.cos(init_rot), -np.sin(init_rot)], [np.sin(init_rot), np.cos(init_rot)]])

        tra = (rot_matrix @ translation.T).T

        delta_theta = np.arctan2(rotation[1, 0], rotation[0, 0])
        print(tra)
        print(delta_theta)



def main():
    rclpy.init()
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
