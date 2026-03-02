import time
import rclpy
import graph.logic.params as params
import graph
from rclpy.node import Node
from graph.logic import moving_path as graph
from interfaces.action import GoTo
from interfaces.msg import RobotState, BMArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, ActionClient
from threading import Lock, Event
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty
from interfaces.msg import Obstacle, ObstacleArray



class SmartGotoNode(Node):
    def __init__(self):
        super().__init__('eureka')
        #important pour le main:
        self.launch_report = self.create_service(Empty, 'init_graph', lambda _, response : response)


        self.pos_x = None
        self.pos_y = None
        self.opp_pos_x = None
        self.opp_pos_y = None
        self.bm_pos = [
                        (0.175, 1.125, 0.0, "BLUE"),
                        (0.175, 1.175, 0.0, "YELLOW"),
                        (0.175, 1.225, 0.0, "BLUE"),
                        (0.175, 1.275, 0.0, "YELLOW"),
                        (0.175, 0.325, 0.0, "YELLOW"),
                        (0.175, 0.375, 0.0, "YELLOW"),
                        (0.175, 0.425, 0.0, "BLUE"),
                        (0.175, 0.475, 0.0, "BLUE"),
                        (1.02586, 0.17510, 1.57, "YELLOW"),
                        (1.07586, 0.17514, 1.57, "BLUE"),
                        (1.12586, 0.17518, 1.57, "YELLOW"),
                        (1.17586, 0.17522, 1.57, "BLUE"),
                        (1.07590, 0.80060, 1.57, "BLUE"),
                        (1.12590, 0.80064, 1.57, "YELLOW"),
                        (1.17590, 0.80068, 1.57, "YELLOW"),
                        (1.22590, 0.80072, 1.57, "BLUE"),
                        (1.77645, 0.80060, 1.57, "YELLOW"),
                        (1.82645, 0.80064, 1.57, "BLUE"),
                        (1.87645, 0.80068, 1.57, "YELLOW"),
                        (1.92645, 0.80072, 1.57, "BLUE"),
                        (1.82649, 0.17510, 1.57, "YELLOW"),
                        (1.87649, 0.17514, 1.57, "YELLOW"),
                        (1.92649, 0.17518, 1.57, "BLUE"),
                        (1.97649, 0.17522, 1.57, "BLUE"),
                        (2.825, 1.125, 0.0, "YELLOW"),
                        (2.825, 1.175, 0.0, "BLUE"),
                        (2.825, 1.225, 0.0, "YELLOW"),
                        (2.825, 1.275, 0.0, "BLUE"),
                        (2.825, 0.325, 0.0, "YELLOW"),
                        (2.825, 0.375, 0.0, "YELLOW"),
                        (2.825, 0.425, 0.0, "BLUE"),
                        (2.825, 0.475, 0.0, "BLUE")]
        self.nids = []
        self.path = []
        self.steps = []
        self.secu_around_bm = params.SECURITY_DISTANCE_BM

        self.goto_result = None
        self.goal_handle = None
        self.mutex = Lock()
        self.group = ReentrantCallbackGroup()
        self.goto_step_event_done = Event()

        self.SmartGotoAction = ActionServer(
            self, GoTo, 'smartgoto', 
            execute_callback=self.smart_goto_callback, callback_group=self.group)
        
        self.GotoAction_client = ActionClient(self, GoTo, 'goto', callback_group=self.group)
        
        self.pos_sub = self.create_subscription(
            RobotState, 'digital_pos', self.pos_callback, 10, callback_group=self.group)
        
        self.opp_pos_sub = self.create_subscription(
            RobotState, 'digital_opp_pos', self.opp_pos_callback, 10, callback_group=self.group)
        
        self.bm_sub = self.create_subscription(
            BMArray,'digital_camera_bm', self.bm_callback, 10, callback_group=self.group)
        
        self.security_area_verif = self.create_publisher(
            ObstacleArray, 'obstacle_area', 10)

    def pos_callback(self, msg):
        with self.mutex:
            self.pos_x= msg.x
            self.pos_y = msg.y
    
    def opp_pos_callback(self, msg):
        with self.mutex:
            self.opp_pos_x= msg.x
            self.opp_pos_y = msg.y

    def bm_callback(self, msg):
        with self.mutex:
            self.bm_pos = []
            for bm in msg.bms:
                self.bm_pos.append((bm.x,bm.y,bm.theta,bm.color))
                
    def smart_goto_callback(self, goal):
        start_odom_time = time.time()
        odom_timeout = 5.
        
        while True:
            with self.mutex:
                if self.pos_x is not None:
                    break
            if time.time() > start_odom_time + odom_timeout:
                self.get_logger().error("Cannot start: No Odometry recieved yet!")
                goal.abort() 
                return GoTo.Result(sucess=False)
            
            if not rclpy.ok() or goal.is_cancel_requested:
                goal.canceled()
                return GoTo.Result(success=False)
            
            time.sleep(.1)
            
            
        if self.pos_x is None:
                self.get_logger().error("Cannot start: No Odometry recieved yet!")
                goal.abort()
                return GoTo.Result(success=False)
        if self.opp_pos_x is None:
                self.get_logger().error("Cannot start: No Lidar_data recieved yet!")
                goal.abort()
                return GoTo.Result(success=False)
        
        if goal.is_cancel_requested:
            goal.canceled()
            self.get_logger().info("error msgs received")
            return GoTo.Result(success=False)

        x_target = goal.request.x 
        y_target = goal.request.y   
        theta_target = goal.request.theta
        self.goal_handle = goal

        self.get_logger().info(f'we are trying to move at {x_target}, {y_target}')
        with self.mutex:
            table = graph.Deplacement_table((self.pos_x, self.pos_y), (self.opp_pos_x, self.opp_pos_y), self.bm_pos, self.nids)

        
        self.steps = table.deplacement((self.pos_x, self.pos_y),(x_target, y_target), theta_target) # donne les steps, le graphe utilisé et les sommets
        self.get_logger().info(f"les étapes pour atteindre le goal: {self.steps}")
        #maintenant il faut envoyer progressivement les étapes au /goto et s'assurer qu'il les fasse
        #creation de la variable d'état
        
        for step in self.steps[1:]:
            self.goto_result = None
            self.goto_step_event_done.clear()

            #creation du goal pour le goto
            current_goto_goal = GoTo.Goal(x=step[0], y=step[1], theta=step[2])

            #envoie de la requête au serveur goto
            self.GotoAction_client.wait_for_server()
            future = self.GotoAction_client.send_goal_async(current_goto_goal, feedback_callback = self.feedback_callback)
            future.add_done_callback(self.goal_response_callback)

            #il faut vérifier que la requête envoyée à goto se termine bien
            while not self.goto_step_event_done.is_set():
                if goal.is_cancel_requested:
                    self.goto_step_event_done.set()
                    goal.canceled()
                    return GoTo.Result(success=False)
                
            if not self.goto_result.success:  
                self.get_logger().info("problem found during the travel")
                goal.abort()
                return GoTo.Result(success = False)
            
        self.get_logger().info("travel complete")
        goal.succeed()
        return GoTo.Result(success=True)        
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info("travel in progress")
        

    def goal_response_callback(self, future):
        self.goto_goal_handle = future.result()

        if not self.goto_goal_handle.accepted:
            self.get_logger().info("Goal refused")
            self.goal_handle.abort()
            return

        self.get_logger().info("Goal accepted")

        result_future = self.goto_goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.goto_step_event_done.set()
        self.goto_result = future.result().result
        self.get_logger().info(
            f"final result: success = {self.goto_result.success}"
        )

    def obst_area_pub(self, obstacles):
        obst_array = ObstacleArray()
        for obst in obstacles:
            ob = Obstacle()
            ob.obst = [value for point in obst for value in point]
            obst_array.obst_list.append(ob)
        
        self.security_area_verif.publish(obst_array)

    
    
def main():
    rclpy.init()
    smart_goto_node = SmartGotoNode()
    executor = MultiThreadedExecutor()
    executor.add_node(smart_goto_node)
    try:
        executor.spin()
    finally:
        executor.shutdown() 
        rclpy.shutdown()

if __name__ == '__main__': 
    main()
