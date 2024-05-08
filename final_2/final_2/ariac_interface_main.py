#!/usr/bin/env python3
import rclpy 
from rclpy.executors import MultiThreadedExecutor

from ariac_interface_util import AriacInterface

def main(args=None):
    """
    Main function to initialize the ROS node and spin the executor.
    """
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    
    interface = AriacInterface("ariac_interface")
    executor.add_node(interface)
    executor.spin()

    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
