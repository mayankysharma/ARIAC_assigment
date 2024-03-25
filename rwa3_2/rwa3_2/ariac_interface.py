#!/usr/bin/env python3

import time
from collections import deque

from rclpy.node import Node
import rclpy 
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# from ship_orders import ShipOrders
from fulfill_orders import CustomTimer
from comp_state import CompetitionState
from read_store_orders import ReadStoreOrders
from ship_orders import ShipOrders

from submit_orders import OrderSubmission  
from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
)

class AriacInterface(Node):
    
    order_topic = "/ariac/orders"
    comp_start_state_service_name = "/ariac/start_competition"
    comp_end_state_service_name = "/ariac/end_competition"
    comp_state_topic_name = "/ariac/competition_state"
    submit_order_service_name = "/ariac/submit_order"

    def __init__(self, node_name):
        super().__init__(node_name)
        group_mutex1 = MutuallyExclusiveCallbackGroup()
        group_reentrant1 = ReentrantCallbackGroup()
        self.order_queue = deque()

         #Competition State object instance
        self.comp_state = CompetitionState(self, AriacInterface.comp_state_topic_name, AriacInterface.comp_start_state_service_name, AriacInterface.comp_end_state_service_name, callback_group=group_reentrant1)
        self._monitor_state = self.create_timer(1, self.monitor_state_callback)
        self.order_submit = OrderSubmission(self,AriacInterface.submit_order_service_name)

        self.ship_order = ShipOrders(self)
        #Read and store order object instance
        self.read_store_orders=ReadStoreOrders(self,AriacInterface.order_topic,self.order_queue,callback_group=group_reentrant1)
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
            order = self.order_queue.popleft()
            ## Now the order if higher priority come after 15 secons our queue will be updated
            # order = self.order_queue.popleft()
            self.get_logger().info(f"Starting the shipping and submitting the order {order.order_id}!!!")
            '''
            Do the task 6 and task 7
            '''
            print("started shipping,",order)
            data = self.ship_order.lock_move_agv(order)
            if data is None:
                self.get_logger().warn("Unable to ship!")
            print("submit order",data)
            self.order_submit.Submit_Order(agv_id=data[1],order_id=data[0])

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