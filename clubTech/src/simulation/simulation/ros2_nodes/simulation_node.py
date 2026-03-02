#!/usr/bin/env python3

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt8
from geometry_msgs.msg import Pose2D, PoseArray
from interfaces.msg import RobotState, BM, BMArray
from interfaces.srv import Do
from simulation.game.game import *


class simulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')
        self.game = Game()
        
        teams = ['blue', 'yellow']
        
        self.digital_wheels_topics = [self.create_subscription(
            Float64MultiArray,
            f'{team}/digital_wheels',
            lambda msg, t=team: self.digital_wheels_callback(msg, t),
            10
        ) for team in teams]
        
        self.digital_actuators_services = [self.create_service(
            Do,
            f'{team}/digital_actuators',
            lambda request, response, t=team: self.digital_actuators_callback(request, response, t)
        ) for team in teams]
        
        # self.digital_actuators_topics = [self.create_subscription(
        #     Do,
        #     f'{team}/digital_actuators',
        #     lambda msg, t=team: self.digital_actuators_callback(msg, t),
        #     10
        # ) for team in teams]

        self.strat_path_topics = [self.create_subscription(
            PoseArray,
            f'{team}/strat_path',
            lambda msg, t=team: self.stratPathCallback(msg, t),
            10
        ) for team in teams]
                
        self.digital_pos_publishers = {team : self.create_publisher(RobotState, f'{team}/digital_pos', 10) for team in teams}
        self.digital_opp_pos_publishers = {team : self.create_publisher(RobotState, f'{team}/digital_opp_pos', 10) for team in teams}
        self.digital_camera_bm_publishers = {team :self.create_publisher(BMArray, f'{team}/digital_camera_bm', 10) for team in teams}
        
        self.robots = {'yellow': self.game.robotYellow, 'blue': self.game.robotBlue}
        

    def stratPathCallback(self, msg, team):
        self.get_logger().info("ICI J'AI LE PAFFF")
        points = msg.poses
        self.robots[team].path = []
        for point in points:
            self.robots[team].path.append(Vector2(point.position.x, point.position.y))
    
    def digital_actuators_callback(self, request, response, team):
        action = request.command & 1
        side = (request.command >> 1) & 1
                
        if action == 0:
            if(self.robots[team].grabbers[side].grab(self.game.boxes)):
                self.robots[team].grabbers[side].sortColor()
                response.success = True
                return response
            else:
                response.success = False
                return response
        else:
            self.robots[team].grabbers[side].release()   
            response.success = True
            return response
            
    # def digital_actuators_callback(self, msg, team):
    #     action = msg.data & 1
    #     side = msg.data >> 1 & 1
        
    #     if action == 0:
    #         self.robots[team].grabbers[side].grab(self.game.boxes)
    #         self.robots[team].grabbers[side].sortColor()
    #     else:
    #         self.robots[team].grabbers[side].release()

            
    def digital_wheels_callback(self, msg, team):
        speed_left, speed_right = msg.data
        self.robots[team].motor_left.setSpeed(speed_left / 60)
        self.robots[team].motor_right.setSpeed(speed_right / 60)
    
    def publish_digital_pos(self, team):
        msg = RobotState()
        msg.x = self.robots[team].body.position.x
        msg.y = self.robots[team].body.position.y
        msg.v = self.robots[team].body.velocity.length
        msg.theta = self.robots[team].body.angle
        msg.omega = self.robots[team].body.angular_velocity
        self.digital_pos_publishers[team].publish(msg)
    
    def publish_digital_opp_pos(self, team):
        msg = RobotState()
        msg.x = self.robots[team].body.position.x
        msg.y = self.robots[team].body.position.y
        msg.theta = self.robots[team].body.angle
        msg.v = self.robots[team].body.velocity.length
        msg.omega = self.robots[team].body.angular_velocity
        self.digital_opp_pos_publishers[team].publish(msg)

    def publish_digital_camera_bm(self, team):
        msg = BMArray()
        for box in self.game.boxes:
            if not box.is_grabed:
                b = BM()
                b.x = box.body.position.x
                b.y = box.body.position.y
                b.theta = box.body.angle
                b.color = box.getTeam()
                msg.bms.append(b)
        self.digital_camera_bm_publishers[team].publish(msg)

    def run(self):
        running = True
        while running:
            running = self.game.updateFromInput()
            
            for team, other_team in [('yellow', 'blue'), ('blue', 'yellow')]:
                self.publish_digital_pos(team)
                self.publish_digital_opp_pos(other_team)
                self.publish_digital_camera_bm(team)
            
            rclpy.spin_once(self, timeout_sec=0)
            self.game.step()
        self.game.quit()