#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
)

from std_srvs.srv import Trigger


class CompetitionState():
    '''
    Class for a competition interface node.

    Args:
        Node (rclpy.node.Node): Parent class for ROS nodes

    Raises:
        KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
    '''

    _competition_states = {
        CompetitionStateMsg.IDLE: 'idle',
        CompetitionStateMsg.READY: 'ready',
        CompetitionStateMsg.STARTED: 'started',
        CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionStateMsg.ENDED: 'ended',
    }
    '''Dictionary for converting CompetitionState constants to strings'''

    def __init__(self, node, topic_name, service_name):

        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )

        self.node = node
        self.topic_name = topic_name
        self.service_name = service_name
        self.competition_started = False

        node.set_parameters([sim_time])
        # Service client for starting the competition
        self._start_competition_client = node.create_client(
            Trigger, service_name)

        # Subscriber to the competition state topic
        self._competition_state_sub = node.create_subscription(
            CompetitionStateMsg,
            self.topic_name,
            self._competition_state_cb,
            10)

        # Store the state of the competition
        self._competition_state: CompetitionStateMsg = None
    
    def _competition_state_cb(self, msg: CompetitionStateMsg):
        f'''Callback for the topic {self.topic_name}
        Arguments:
            msg -- CompetitionState message
        '''

        self.node.get_logger().info('Waiting for competition to be ready')
        # Log if competition state has changed
        if self._competition_state != msg.competition_state:
            state = CompetitionState._competition_states[msg.competition_state]
            self.node.get_logger().info(f'Competition state is: {state}', throttle_duration_sec=1.0)

        self._competition_state = msg.competition_state


        if self._competition_state == CompetitionStateMsg.STARTED:
            return 
        # Wait for competition to be ready
        if self._competition_state != CompetitionStateMsg.READY:
            return 

        self.node.get_logger().info('Competition is ready. Starting...')

        # Check if service is available
        if not self._start_competition_client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().error(f'Service \'{self.service_name}\' is not available.')
            return 

        # Create trigger request and call starter service
        request = Trigger.Request()
        future = self._start_competition_client.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self.node, future)

        if future.result().success:
            self.get_logger().info('Started competition.')
            self.competition_started = True
            return 
        else:
            self.get_logger().warn('Unable to start competition')
            return 


