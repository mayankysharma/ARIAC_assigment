#!/usr/bin/env python3


from rclpy.node import Node
import rclpy 
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


from comp_state import CompetitionState
from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
)

class AriacInterface(Node):
    
    order_topic = "/ariac/orders"
    comp_start_state_service_name = "/ariac/start_competition"
    comp_state_topic_name = "/ariac/competition_state"

    def __init__(self, node_name):
        super().__init__(node_name)
        group_mutex1 = MutuallyExclusiveCallbackGroup()
        group_reentrant1 = ReentrantCallbackGroup()

        self.comp_state = CompetitionState(self, AriacInterface.comp_state_topic_name, AriacInterface.comp_start_state_service_name, callback_group=group_reentrant1)
        self._monitor_state = self.create_timer(0, self.monitor_state_callback, callback_group=group_reentrant1)

    def monitor_state_callback(self):

        if self.comp_state.competition_started and not self.comp_state.competition_ended:
            print("STARTED Do other task now!!!")


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    
    interface = AriacInterface("ariac_interface")
    executor.add_node(interface)
    executor.spin()

    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()