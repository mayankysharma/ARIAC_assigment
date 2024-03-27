#!/usr/bin/env python3

import time
from collections import deque

from rclpy.node import Node
import rclpy 
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# from ship_orders import ShipOrders
from custom_timer import CustomTimer
from comp_state import CompetitionState
from read_store_orders import ReadStoreOrders
from ship_orders import ShipOrders

from submit_orders import OrderSubmission  
from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
)

class AriacInterface(Node):
    """
    Class representing the interface for managing ARIAC competition tasks.
    """
    
    order_topic = "/ariac/orders"
    comp_start_state_service_name = "/ariac/start_competition"
    comp_end_state_service_name = "/ariac/end_competition"
    comp_state_topic_name = "/ariac/competition_state"
    submit_order_service_name = "/ariac/submit_order"

    def __init__(self, node_name):
        """
        Initialize the AriacInterface node.

        :param node_name: Name of the ROS node.
        """
        super().__init__(node_name)
        group_mutex1 = MutuallyExclusiveCallbackGroup()
        group_reentrant1 = ReentrantCallbackGroup()
        self.order_queue = deque()

         #Competition State object instance
        self.comp_state = CompetitionState(self, AriacInterface.comp_state_topic_name, AriacInterface.comp_start_state_service_name, AriacInterface.comp_end_state_service_name, callback_group=group_reentrant1)
        self._monitor_state = self.create_timer(1, self.monitor_state_callback)
        self.order_submit = OrderSubmission(self,AriacInterface.submit_order_service_name,group_reentrant1)

        self.ship_order = ShipOrders(self,group_reentrant1)
        #Read and store order object instance
        self.read_store_orders=ReadStoreOrders(self,AriacInterface.order_topic,self.order_queue,callback_group=group_reentrant1)
        self.custom_timer = CustomTimer(self)

    def monitor_state_callback(self):
        """
        Callback function to monitor the state of the competition.
        """
        if self.comp_state.competition_started and not self.comp_state.competition_ended:
            self.fulfill_orders()

    def fulfill_orders(self):
        """
        Method to fulfill orders during the competition.
        """
        if self.custom_timer.check_wait_flag():
            return

        self.get_logger().info("Done waiting, taking order now, Delay 15 secs for new order!!")

        if len(self.order_queue)>0:
            try:
                if self.custom_timer.check_delay_flag():
                    return
                self.get_logger().info("Got the Order!!")
                order = self.order_queue.popleft()

                self.get_logger().info(f"Starting the shipping and submitting the order {order.order_id}!!!")

                data = self.ship_order.lock_move_agv(order)
                if data is None:
                    self.get_logger().warn("Unable to ship!")

                while not self.order_submit.Submit_Order(agv_num=data[1],order_id=data[0]): continue
            except Exception as e:
                self.get_logger().warn(f"Unable to ship and submit the order because of {e}!")
        else:
            if self.comp_state.all_orders_recieved:
                self.comp_state.competition_ended = True

        self.custom_timer.reset_flags()


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
