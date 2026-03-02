import rclpy
from enum import Enum                                                                                    
from interfaces.action import GoTo
from interfaces.srv  import Do
from interfaces.msg import RobotState
from std_msgs.msg import Float64, String
from rclpy.node import Node   
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, Pose
from std_srvs.srv import Empty
from strat.aco import getACO, robotState, types, nid

class State(Enum):
    Inactive = 1
    Available = 2,
    Moving = 3,
    Catching = 4,
    Releasing = 5,
                                                   
class stratNode(Node):
    def __init__(self):
        super().__init__('strat')
        self.launch_report = self.create_service(Empty, 'init_strat', lambda _, response : response)
        self.start_game_subsriber = self.create_subscription(String, 'start_game', self.start_game_callback, 10)
        self.path_pub = self.create_publisher(PoseArray, "strat_path", 10)
        
        self.do = self.create_client(Do, 'do')
        self.create_subscription(RobotState, "digital_pos",  self.posCallback, 10)
        self.goto_action_client = ActionClient(self, GoTo, 'smartgoto')
        
        #defs aco
        self.state = State.Inactive
        self.aco_gen, self.refresh_aco, self.update_aco_pos = getACO(self.get_namespace()[1:])
        self.ids, self.destinations = next(self.aco_gen)
        
        self.x , self.y, self.theta = 0., 0., 0.
        self.sendPath()
        self.timer = self.create_timer(0.5, self.stratLoop)
    
    def sendPath (self):
        msg = PoseArray()
        for (x, y), _ in self.destinations:
            pos = Pose()
            pos.position.x = float(x)
            pos.position.y = float(y)
            pos.position.z = 0.
            pos.orientation.w = 1.
            msg.poses.append(pos)
        msg.poses.pop(0)
        self.path_pub.publish(msg)
        
    def start_game_callback(self, msg):
        self.move(self.destinations[1])
        
    def posCallback(self, msg):
        self.x, self.y, self.theta = msg.x, msg.y, self.theta
        self.update_aco_pos([self.x, self.y])
        
    def move(self, dest):
        """
        moves the robot to requested destination, 
        if successfull, it puts the robot in the correct state (catching/releasing)
        in case of failure, the robot becomes available
        """
        (x, y), theta = dest
        goal = GoTo.Goal()
        goal.x = float(x)
        goal.y = float(y) 
        goal.theta = float(theta)
        
        self.state = State.Moving
        self.goto_action_client.wait_for_server()
        
        send_goal_future = self.goto_action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.moveResponseCallback)
    
    def moveResponseCallback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal refused")
            goal_handle.abort()
            self.state = State.Available
            return
        
        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.moveGetResultCallback)
    
    def moveGetResultCallback(self, future):
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal reached successfully")
            
             
            if (types[self.ids[1]] == 0 or types[self.ids[1]] == 4) and robotState.canGrab(): 
                self.grab()
                self.state = State.Catching 
            elif (types[self.ids[1]] == 1 or types[self.ids[1]] == 6) and robotState.canDrop():
                self.release()
                self.state = State.Releasing
            elif types[self.ids[1]] == 3:
                self.state = State.Inactive
            else:
                self.state = State.Available
        else:
            self.state = State.Available
        
    def stratLoop(self):
        if(self.state == State.Available):
            self.destinations.pop(0)
            self.ids.pop(0)
            
            self.sendPath()
            self.move(self.destinations[1])
            return
        if(self.state == State.Inactive):
            return
        if(self.state == State.Moving):
            #waits for move completion
            return
        if(self.state == State.Catching):
            self.get_logger().info(f"{self.state}")
            #waits for callback completion
            return
        if(self.state == State.Releasing):
            self.get_logger().info(f"{self.state}")
            #waits for callback completion
            return
        
        
    def sendDo (self, data: int):
        request = Do.Request()
        request.command = data
        self.do.call_async(request).add_done_callback(self.handleDone)
        
    def handleDone (self, future):
        achieved = future.result()
        robotState.run(self.ids[1], achieved)
        self.ids, self.destinations = next(self.aco_gen)
        self.sendPath()
        self.state = State.Available

    def grab(self, back = False):
        self.sendDo(back << 1)
        self.get_logger().info("Grab command sent")  

    def release(self, back = False):
        self.sendDo((back << 1) | 1)
        self.get_logger().info("Release command sent")
            
def main():
    rclpy.init()
    node = stratNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()