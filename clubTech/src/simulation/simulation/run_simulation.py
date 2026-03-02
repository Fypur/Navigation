#!/usr/bin/env python3

from simulation.ros2_nodes.simulation_node import simulationNode
import rclpy
import random

def main():
    random.seed(10)

    rclpy.init()
    sim_node = simulationNode()

    try:
        sim_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        sim_node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()
        