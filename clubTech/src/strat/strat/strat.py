import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool, Float64, String, UInt8
from geometry_msgs.msg import Pose2D, PoseArray, Pose
from enum import Enum
from std_srvs.srv import Empty

import uuid
from typing import List
from interfaces.msg import RobotState, BM, BMArray
import numpy as np
from strat.aco import getACO, kAtticsPosition, kGrabBoxPositionAngle, robotState

from interfaces.action import GoTo
from interfaces.srv import Do

class State(Enum):
    START_MATCH = 1
    DECISION_MAKING = 2
    MOVE_TO_GOAL = 3
    GRAB_KAPLA = 4
    RELEASE_KAPLA = 5
    END_MATCH = 6


class StratFSM(Node):
    def __init__(self):
        super().__init__("strat_fsm")
        #important pour le main:
        self.launch_report = self.create_service(Empty, 'init_strat', lambda _, response : response)
        self.start_game_subsriber = self.create_subscription(String, 'start_game', self.start_game_callback, 10)
        
        self.state = State.START_MATCH
        
        self.clock_value = 0.0
        
        
        # Flags to know if we already executed the action for that state
        self.mission_started = False
        

        self.curr_pos: np.ndarray = np.array([0.,0.]) # TODO
        
        self.curr_orientation = None
        

        # ! duplicates
        self.pos_available_kaplas: List[list] = [[True, np.array(p)] for p, _ in kGrabBoxPositionAngle] # Starting
        self.pos_available_gardens: List[list] = [[True, np.array(p)] for p, _ in kAtticsPosition] 
        self.gardens_to_be_returned: List[tuple] = [(False, np.array(p)) for p, _ in kAtticsPosition]
        

        self.req_id = None
        

    def start_game_callback(self, msg):
        self.do = self.create_client(Do, 'do')
        
        self.do = self.create_client(Do, 'do')
        self.create_subscription(RobotState, "digital_pos",  self.update_pos_cb, 10)
        self.create_subscription(Float64, "clock_value", self.clock_cb, 10)

        # self.create_subscription(Float64MultiArray, "update_directions", self.update_directions_cb, 10)
        # dist = ACO.getDist(np.array(kAtticsPosition + kBoxStartPositionAngle))
        # self.aco = ACO(endPoints=([0], []), n=len(dist), evalRoute=lambda route: sum(dist[route[i], route[i + 1]] for i in range(len(route) - 1)))

        self.acoGen, self.refreshACO, self.updateACOPos = getACO(self.get_namespace())
        self.ids, self.directions = next(self.acoGen)
        self.path_pub = self.create_publisher(PoseArray, "strat_path", 10)
        self.move_client = ActionClient(self, GoTo, "goto")
        
        self.sendPath()
        self.get_logger().info("STRAT FSM READY.")
        self.create_timer(0.1, self.fsm_step) # TIMER EN SECONDES
        
        
    def sendPath (self):
        msg = PoseArray()
        for (x, y), _ in self.directions:
            pos = Pose()
            pos.position.x = float(x)
            pos.position.y = float(y)
            pos.position.z = 0.
            pos.orientation.w = 1.
            msg.poses.append(pos) # type: ignore
        # msg = [Pose(position = {"x": x, "y": y, "z": 0 }, orientation = {"w": 1.}) for x, y in self.directions]
        self.path_pub.publish(msg)

    def refreshPath (self):
        self.refreshACO()
        ids, directions = next(self.acoGen)
        
        # self.get_logger().info(f"ids: {self.ids} - {ids}")
        # if self.ids == ids:
        #     return
        
        self.get_logger().info("updated")
        
        self.ids, self.directions = ids, directions
        self.refreshACO()
        self.sendPath()
    
    def update_pos_cb(self, msg: Pose2D):
        # Pose 2D classique : (x, y, theta)
        self.curr_pos = np.array([msg.x, msg.y])
        self.updateACOPos(self.curr_pos)
        self.refreshPath()
        self.curr_orientation = msg.theta

    def start_cb(self, msg: Bool):
        if msg.data and self.state == State.START_MATCH:
            self.get_logger().info("MATCH START")
            self.state = State.DECISION_MAKING

    def clock_cb(self, msg: Float64):
        self.clock_value = msg.data
        if self.clock_value >= 90.0:
            self.get_logger().warn("TIME OVER -> RETURN TO START")
            self.reset_flags()
            self.state = State.END_MATCH
    
    def kapla_released_cb(self, msg):
        if msg.data == self.req_id:
            self.get_logger().info("Released Kapla!")
        else:
            self.get_logger().info("False req id!")
        
        self.pos_available_gardens[self.idx_gardens] = False, self.pos_available_gardens[self.idx_gardens][1]

        self.state = State.DECISION_MAKING
        self.reset_flags()

    def kapla_grabed_cb(self, msg):
        if msg.data == self.req_id:
            self.get_logger().info("Grabed Kapla!")
        else:
            self.get_logger().info("False req id!")
        
        self.pos_available_kaplas[self.idx_kaplas] = False, self.pos_available_kaplas[self.idx_kaplas][1]

        self.state = State.DECISION_MAKING
        self.reset_flags()

    
    def update_directions_cb(self, msg):
        self.directions = msg.data # TODO

    def fsm_step(self):
        if self.state == State.START_MATCH:
            self.state = State.DECISION_MAKING

        if self.state == State.DECISION_MAKING:
            self.handle_decision()
            self.reset_flags()

        elif self.state == State.MOVE_TO_GOAL:
            if not self.mission_started:
                self.mission_started = True
                self.start_move_action()

        elif self.state == State.GRAB_KAPLA:
            if not self.mission_started:
                self.mission_started = True
                self.grab(False)
        
        elif self.state == State.RELEASE_KAPLA:
            if not self.mission_started:
                self.mission_started = True
                self.release(False)

    def reset_flags(self):
        self.mission_started = False

    def verify_near_by(self, positions):
        THRESHOLD_DIST = .02
        cmp = (np.linalg.norm(self.curr_pos - pos) < THRESHOLD_DIST for available, pos in positions if available)
        idx = next((i for i, x in enumerate(cmp) if x), None) 
        return idx

    def handle_decision(self):
        idx_kaplas = self.verify_near_by(self.pos_available_kaplas)
        idx_gardens = self.verify_near_by(self.pos_available_gardens)
        if robotState.canGrab() and idx_kaplas is not None:
            self.state = State.GRAB_KAPLA
            self.idx_kaplas = idx_kaplas
        
        if robotState.canDrop() and idx_gardens is not None: 
            self.state = State.RELEASE_KAPLA
            self.idx_gardens = idx_gardens

        if self.state == State.DECISION_MAKING:
            if not self.directions:
                self.get_logger().info(f"NO DIRECTIONS SETTED -> WAITING")
                return
            
            self.directions.pop(0)
            self.ids.pop(0)
            
            self.get_logger().info(f"Decision: MOVE to ({self.directions[0][0][0]:.2f}, {self.directions[0][0][1]:.2f})")
            self.state = State.MOVE_TO_GOAL

    def start_move_action(self):
        if not self.move_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("MoveToGoal server unavailable")
            self.state = State.DECISION_MAKING
            return
        
        (x, y), theta = self.directions[0]

        goal = GoTo.Goal()
        goal.x = float(x)
        goal.y = float(y)
        goal.theta = float(theta)

        self.get_logger().info(f"Sending MoveToGoal action {goal}")

        future = self.move_client.send_goal_async(
            goal,
            feedback_callback=self.move_feedback
        )
        future.add_done_callback(self.move_response)

    def move_feedback(self, msg):
        self.get_logger().info(
            f"Position percentage: ({msg.feedback.percentage_distance}"  # {pos.current_x:.2f}, {pos.current_y:.2f})"
        )

    def move_response(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Move goal rejected -> DECISION_MAKING")
            self.state = State.DECISION_MAKING
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.move_result)

    def move_result(self, future):
        result = future.result().result

        if result.success:
            self.get_logger().info("Goal reached -> DECISION_MAKING")
        else:
            self.get_logger().warn("Move failed -> DECISION_MAKING")
        self.state = State.DECISION_MAKING
        self.reset_flags()

    def sendDo (self, data: int):
        request = Do.Request()
        request.command = data
        self.do.call_async(request).add_done_callback(self.handleDone)

    def handleDone (self, future):
        achieved = future.result()
        robotState.run(self.ids[0], achieved)
        self.refreshPath()
        self.get_logger().info('on a la réponse')

        if self.state == State.GRAB_KAPLA:
            self.pos_available_kaplas[self.idx_kaplas][0] = False
        else:
            self.pos_available_gardens[self.idx_gardens][0] = False
        self.state = State.DECISION_MAKING

    def grab(self, back = False):
        self.sendDo(back << 1)
        self.get_logger().info("Grab command sent")  

    def release(self, back = False):
        self.sendDo((back << 1) | 1)
        self.get_logger().info("Release command sent")


def main(args=None):
    rclpy.init(args=args)
    node = StratFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()