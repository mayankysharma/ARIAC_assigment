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
    Class for managing competition state.

    Args:
        node (rclpy.node.Node): ROS node instance.
        topic_name (str): Name of the competition state topic.
        start_service (str): Name of the service to start the competition.
        end_service (str): Name of the service to end the competition.
        callback_group: Callback group for this class.

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

    def __init__(self, node, topic_name, start_service, end_service, callback_group):
        '''
        Initialize CompetitionState.

        Args:
            node (rclpy.node.Node): ROS node instance.
            topic_name (str): Name of the competition state topic.
            start_service (str): Name of the service to start the competition.
            end_service (str): Name of the service to end the competition.
            callback_group: Callback group for this class.
        '''

        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )

        self.node = node
        self.topic_name = topic_name
        self.start_service = start_service
        self.end_service = end_service

        node.set_parameters([sim_time])
        # Service client for starting the competition
        self._start_competition_client = node.create_client(
            Trigger, self.start_service)

        self._end_competition_client = node.create_client(
            Trigger, self.end_service)

        # Subscriber to the competition state topic
        self._competition_state_sub = node.create_subscription(
            CompetitionStateMsg,
            self.topic_name,
            self._competition_state_cb,
            10,
            callback_group=callback_group)

        # Store the state of the competition
        self._competition_state: CompetitionStateMsg = None
        # Flag for checking if the competition state is ended
        self.competition_ended = False 
        
        # Flag for checking if the state has changed to start
        self.competition_started = False

        # Flag for checking if all orders are received
        self.all_orders_recieved = False

    def _competition_state_cb(self, msg: CompetitionStateMsg):
        '''
        Callback function for the competition state topic.

        Args:
            msg (CompetitionStateMsg): Message received on the competition state topic.
        '''

        # Log if competition state has changed
        if self._competition_state != msg.competition_state:
            state = CompetitionState._competition_states[msg.competition_state]
            self.node.get_logger().info(f'Competition state is: {state}', throttle_duration_sec=1.0)

        self._competition_state = msg.competition_state

        if self._competition_state == CompetitionStateMsg.STARTED or self._competition_state == CompetitionStateMsg.ENDED:
            return

        if self._competition_state == CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE:
            self.all_orders_recieved = True
            if self.competition_ended:
                self.terminate_competition() 

        if self._competition_state == CompetitionStateMsg.READY:
            self.node.get_logger().info('Waiting for competition to be ready')
        
        # Wait for competition to be ready
        if self._competition_state == CompetitionStateMsg.READY and not self.competition_started:
            self.start_competition()

    def terminate_competition(self):
        '''
        Terminate the competition.
        '''

        self.node.get_logger().info('Terminating Competition')

        # Check if service is available
        while not self._end_competition_client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().error(f'Service \'{self.end_service}\' is not available, waiting...')

        # Create trigger request and call starter service
        self.node.get_logger().info("calling request for terminating")
        request = Trigger.Request()

        response = self._end_competition_client.call(request)

        if response.success:
            self.node.get_logger().info('Competition Ended.')
            return 
        else:
            self.node.get_logger().warn('Unable to terminate competition')
            return 

    def start_competition(self):
        '''
        Start the competition.
        '''
        
        self.node.get_logger().info('Competition is ready. Starting...')

        # Check if service is available
        while not self._start_competition_client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().error(f'Service \'{self.start_service}\' is not available, waiting...')

        # Create trigger request and call starter service
        self.node.get_logger().info("calling request for starting")
        request = Trigger.Request()

        response = self._start_competition_client.call(request)

        if response.success:
            self.node.get_logger().info('Started competition.')
            self.competition_started = True
            return 
        else:
            self.node.get_logger().warn('Unable to start competition')
            return 

