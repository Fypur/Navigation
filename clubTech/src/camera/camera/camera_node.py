import cv2
import numpy  as np
import rclpy

from utils import *
from rclpy.node import Node
from game_parameters import *

from geometry_msgs.msg import Pose2D
from interfaces.msg import BMArray, BM
from std_msgs.msg import String

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Initialize video capture
        self.video_capture = cv2.VideoCapture(camera_index)
        self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # Load camera calibration and estimate camera pose
        self.calibration = load_camera_calibration()
        self.camera_pose = get_camera_position(n_obs=n_obs, obs_limit=obs_limit, calibration=self.calibration)
        
        # Publishers
        self.camera_self_publisher = self.create_publisher(Pose2D, '/camera_self', 10) #Publisher for own robot pose
        self.camera_opp_publisher = self.create_publisher(Pose2D, '/camera_opp', 10) #Publisher for opponent robot pose
        self.camera_bm_publisher = self.create_publisher(BMArray,'/camera_bm', 10) #Publisher for BM positions
        
        # Subscriber for team color
        self.color_subscriber = self.create_subscription(String, '/color', self.color_callback, 10)
        self.color = None
        
        self.get_logger().info("Camera node initialized.")

    # Callback to update team color
    def color_callback(self, msg):
        self.color = msg.data
        self.get_logger().info(f"Updated team color: {self.color}")
        
    # Callback to publish own position
    def publish_self_pose(self, position):
        msg = Pose2D()
        msg.x = position[0]
        msg.y = position[1]
        msg.theta = position[2]
        self.camera_self_publisher.publish(msg)
     
    # Callback to publish opponent position
    def publish_opp_pose(self, position):
        msg = Pose2D()
        msg.x = position[0]
        msg.y = position[1]
        msg.theta = position[2]
        self.camera_opp_publisher.publish(msg)
        
    # Callback to publish BM positions
    def publish_bm_pose(self, bm_positions, bm_orientations):
        msg = BMArray()

        for color, positions in bm_positions.items():
            orientations = bm_orientations[color]

            for pos, ori in zip(positions, orientations):
                bm = BM()
                bm.color = color
                bm.x = float(pos[0])
                bm.y = float(pos[1])
                bm.theta = float(ori)
                msg.bms.append(bm)

        self.camera_bm_publisher.publish(msg)

        
def main():
    
    # Creatting the ROS2 node
    rclpy.init()
    camera_node = CameraNode()
    
    # Opening camera
    cap = None
    while cap is None or not cap.isOpened():
        cap = open_camera()
    
    calibration = load_camera_calibration(calibration_file)
    
    camera_pose = None
    while camera_pose is None:
        camera_pose = get_camera_position(n_obs=n_obs, obs_limit=obs_limit, calibration=calibration)
    
    # Main loop
    while rclpy.ok():
        
        #TODO: take into account the height of the robot in pose estimation
        # Finding blue markers
        blue_coords, blue_yaws = find_marker_id(capture = cap, marker_ids = blue_robot_ids, marker_size = robot_marker_size,
                                                camera_pose = camera_pose, calibration = calibration, height= robot_height)

        # Finding yellow markers
        yellow_coords, yellow_yaws = find_marker_id(capture = cap, marker_ids = yellow_robot_ids, marker_size = robot_marker_size,
                                                    camera_pose = camera_pose, calibration = calibration, height= robot_height)

        # Publishing positions
        if camera_node.color == "blue":
            if blue_coords:
                camera_node.publish_self_pose((blue_coords[0][0], blue_coords[0][1], blue_yaws[0]))
            if yellow_coords:
                camera_node.publish_opp_pose((yellow_coords[0][0], yellow_coords[0][1], yellow_yaws[0]))
        
        elif camera_node.color == "yellow":
            if yellow_coords:
                camera_node.publish_self_pose((yellow_coords[0][0], yellow_coords[0][1], yellow_yaws[0]))
            if blue_coords:
                camera_node.publish_opp_pose((blue_coords[0][0], blue_coords[0][1], blue_yaws[0]))

        # Finding BM markers
        bm_poses, bm_orientations  = find_bms(capture = cap, camera_pose = camera_pose, calibration = calibration,
                                              IDs = set([blue_id, yellow_id, empty_id]), marker_size = bm_marker_size, height= bm_height)

        # Publishing BM positions
        camera_node.publish_bm_pose(bm_poses, bm_orientations)

        # Spin once
        rclpy.spin_once(camera_node)
        