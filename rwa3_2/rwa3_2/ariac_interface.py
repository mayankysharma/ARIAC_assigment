#!/usr/bin/env python3

import time
from collections import deque

from rclpy.node import Node
import rclpy 
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# from ship_orders import ShipOrders
# from read_store_orders import ReadStoreOrders
from fulfill_orders import CustomTimer
from comp_state import CompetitionState
from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
)

class AriacInterface(Node):
    
    order_topic = "/ariac/orders"
    comp_start_state_service_name = "/ariac/start_competition"
    comp_end_state_service_name = "/ariac/end_competition"
    comp_state_topic_name = "/ariac/competition_state"

    def __init__(self, node_name):
        super().__init__(node_name)
        group_mutex1 = MutuallyExclusiveCallbackGroup()
        group_reentrant1 = ReentrantCallbackGroup()
        self.order_queue = deque([1,2])

        self.comp_state = CompetitionState(self, AriacInterface.comp_state_topic_name, AriacInterface.comp_start_state_service_name, AriacInterface.comp_end_state_service_name, callback_group=group_reentrant1)
        self._monitor_state = self.create_timer(1, self.monitor_state_callback)
        
        self.custom_timer = CustomTimer(self)

    def monitor_state_callback(self):

        if self.comp_state.competition_started and not self.comp_state.competition_ended:
            # print("STARTED Do other task now!!!")
            self.fufill_orders()

    def fufill_orders(self):

        # self.get_logger().info('Waiting for Orders!!')
         
        if self.custom_timer.check_wait_flag():
            return

        self.get_logger().info("Done waiting, taking order now!!")

        if len(self.order_queue)>0:

            ## wait for 15 seconds before processing the order and moving to task 6
            if self.custom_timer.check_delay_flag():
                return

            ## Now the order if higher priority come after 15 secons our queue will be updated
            # order = self.order_queue.popleft()
            self.get_logger().info("Starting the shipping and submitting the order!!!")
            '''
            Do the task 6 and task 7
            '''
            order = self.order_queue.popleft()
        else:
            if self.comp_state.all_orders_recieved:
                self.comp_state.competition_ended = True

        self.custom_timer.reset_flags()


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