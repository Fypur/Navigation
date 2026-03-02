import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Float32, UInt8, Float64MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from interfaces.srv import Do
from std_srvs.srv import Empty
from numpy import pi

radius = 0.035

class ActionActuator(Node):
    def __init__(self):
        super().__init__('actuators')
        self.launch_report = self.create_service(Empty, 'init_actuators', lambda _, response : response)
        
        self.cb_group = ReentrantCallbackGroup()
        
        self.digital_actuator_client = self.create_client(
            Do, 'digital_actuators', callback_group=self.cb_group
        )

        self.do_service = self.create_service(
            Do, 'do', self.do_callback, callback_group=self.cb_group
        )
        
        self.wheelsPublisher = self.create_publisher(
            Float64MultiArray, 'wheels', 10
        )
        
    async def do_callback(self, request, response):
        action = request.command & 1
        side = request.command >> 1 & 1
        
        #on s'avance un peu pour ramasser les kaplas
        if action == 0:
            t = 1.0
            dist = 0.12
            omega = dist * 60 / (t * 2 * pi * radius)

            msg = Float64MultiArray()
            msg.data = [float(omega), float(omega)]
            self.wheelsPublisher.publish(msg)

            time.sleep(t) 

            msg.data = [0.0, 0.0]
            self.wheelsPublisher.publish(msg)

        digital_request = Do.Request()
        digital_request.command = request.command
        result = await self.digital_actuator_client.call_async(digital_request)
        
        #on revient vers l'arrière dans le cas du dépos
        if action == 1:
            t = 1.0
            dist = 0.12
            omega = dist * 60 / (t * 2 * pi * radius)
            
            msg = Float64MultiArray()
            msg.data = [-float(omega), -float(omega)]
            self.wheelsPublisher.publish(msg)
            
            time.sleep(t) 
            
            msg.data = [0.0, 0.0]
            self.wheelsPublisher.publish(msg)
        
        return result

def main():
    rclpy.init()
    node = ActionActuator()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try :
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()