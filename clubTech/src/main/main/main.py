#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
from launcher import nodesToLaunch

class Main(Node):
    def __init__(self):
        super().__init__('main')
        
        self.screen = self.create_publisher(String, 'print', 10)
        self.strat_publisher = self.create_publisher(String, 'start_game', 10)
        #self.initNode('screen')
        self.pending_nodes = [name for name, _, asks in nodesToLaunch if asks]        
        self.create_timer(1.0, self.launchServices)
        
    def launchServices(self):
        if hasattr(self, '_launched'): return
        self._launched = True
        
        for name in self.pending_nodes[:]:
            self.get_logger().info(f"{name}: starting")
            self.initNode(name) 
    
    def finish_init(self):
        msg = String()
        msg.data = self.get_namespace()
        self.get_logger().info("--- TOUS LES NODES SONT PRETS : Lancement du robot ---")
        self.strat_publisher.publish(msg)


    def showToScreen (self, message: str):
        msg = String()
        msg.data = message
        self.screen.publish(msg)

    def initNode (self, name: str):
        client = self.create_client(Empty, f'init_{name}')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service init_{name} introuvable (timeout)")
            return
                
        def callback(future):
            try:
                future.result()
                self.get_logger().info(f"{name}: ready")
                self.showToScreen(f"{name}: ready")
                
                if name == "screen":
                    self.launchServices
                    
                if name in self.pending_nodes:
                    self.pending_nodes.remove(name)
                if len(self.pending_nodes) == 0:
                    self.finish_init()
            except Exception as e:
                self.get_logger().error(f"error calling {name}: {e}")

        client.call_async(Empty.Request()).add_done_callback(callback)


def main(args=None):
    rclpy.init(args=args)
    node = Main()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
