#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
)

from std_srvs.srv import Trigger


class AriacInterface(rclpy.node.Node):

    _competition_states = {
        CompetitionStateMsg.IDLE: 'idle',
        CompetitionStateMsg.READY: 'ready',
        CompetitionStateMsg.STARTED: 'started',
        CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionStateMsg.ENDED: 'ended',
    }

    comp_state_topic_name = "/ariac/competition_state"
    start_comp_service_name = "/ariac/start_competition"

    def __init__(self, node_name):
        super(AriacInterface, self).__init__(node_name)

        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )

        self.set_parameters([sim_time])

        self.sub = self.create_subscription(
            CompetitionStateMsg,
            AriacInterface.comp_state_topic_name,
            self.comp_callback,
            10)
        
        self._comp_state = None
        self._comp_started=False

        self._start_comp_client = self.create_client(Trigger, AriacInterface.start_comp_service_name)


    def comp_callback(self, msg):
        if self._comp_state != msg.competition_state:
            state = AriacInterface._competition_states[msg.competition_state]
            self.get_logger().info(f"State Changed to {state}")
            self._comp_state = msg.competition_state
        if self._comp_started:
            return
        while self._comp_state != "ready":
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                return
        self._comp_started = True
        # if not self._comp_started and self._comp_state == "ready":
        self.start_competition() # to start the competition

    def start_competition(self):

        while not self._start_comp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        
        self._request= Trigger.Request()

        # Call the service asynchronously
        future = self._start_comp_client.call_async(self._request) # request is empty just need a response.

        # Add a callback function to be called when the future is complete
        future.add_done_callback(self.wait_comp_callback)

    def wait_comp_callback(self, future):
        """
        Callback function for the future object

        Args:
            future (Future): A future object
        """
        details = future.result()
        if details.success:
            self.get_logger().info(f"Successfully Started!!")
        else:
            self.get_logger().info(f"{details.message}")
   

def main(args=None):
    rclpy.init(args=args)

    comp_state_sub = AriacInterface("ariac_interface")

    rclpy.spin(comp_state_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    comp_state_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()