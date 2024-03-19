#!/usr/bin/env python3


from rclpy.node import Node
import rclpy 

from comp_state import CompetitionState

class AriacInterface(Node):
    
    order_topic = "/ariac/orders"
    comp_start_state_service_name = "/ariac/start_competition"
    comp_state_topic_name = "/ariac/competition_state"

    def __init__(self, node_name):
        super().__init__(node_name)
        self.comp_state = CompetitionState(self, AriacInterface.comp_state_topic_name, AriacInterface.comp_start_state_service_name)

        while True:
            self.get_logger().info(f'Competition is started : {self.comp_state.competition_started}')




def main(args=None):
    rclpy.init(args=args)
    interface = AriacInterface("ariac_interface")
    rclpy.spin(interface)
    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()