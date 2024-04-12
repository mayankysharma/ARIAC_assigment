#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ariac_msgs.msg import (
    Order as OrderMsg,
    AGVStatus as AGVStatusMsg,
    AssemblyTask as AssemblyTaskMsg,
    Part as PartMsg
)
from utils import(
    Order
)

class ReadStoreOrders():
    '''
    Class for subscribing to order messages and storing them in a queue.

    Args:
        node (rclpy.node.Node): The ROS node.
        topic_name (str): Name of the topic where orders are published.
        order_queue (collections.deque): Queue to store orders.
        callback_group: Callback group for this class.
    '''

    _AGV_destinations = {
        AGVStatusMsg.KITTING: 'kitting station',
        AGVStatusMsg.ASSEMBLY_FRONT: 'front assembly station',
        AGVStatusMsg.ASSEMBLY_BACK: 'back assembly station',
        AGVStatusMsg.WAREHOUSE: 'warehouse',
    }
    '''Dictionary for converting AGVDestination constants to strings'''

    _Assembly_stations = {
        AssemblyTaskMsg.AS1: 'assembly station 1',
        AssemblyTaskMsg.AS2: 'assembly station 2',
        AssemblyTaskMsg.AS3: 'assembly station 3',
        AssemblyTaskMsg.AS4: 'assembly station 4',
    }

    _color_of_parts = {
        PartMsg.RED: 'red',
        PartMsg.BLUE: 'blue',
        PartMsg.GREEN: 'green',
        PartMsg.ORANGE: 'orange',
        PartMsg.PURPLE: 'purple',
    }
    '''Dictionary for converting Part color constants to strings'''

    _part_color_symbol = {
        PartMsg.RED: '🟥',
        PartMsg.BLUE: '🟦',
        PartMsg.GREEN: '🟩',
        PartMsg.ORANGE: '🟧',
        PartMsg.PURPLE: '🟪',
    }

    _type_of_parts = {
        PartMsg.BATTERY: 'battery',
        PartMsg.PUMP: 'pump',
        PartMsg.REGULATOR: 'regulator',
        PartMsg.SENSOR: 'sensor',
    }
    '''Dictionary for converting Part type constants to strings'''

    def __init__(self, node, topic_name1, order_queue, callback_group):
        '''
        Initialize ReadStoreOrders.

        Args:
            node (rclpy.node.Node): The ROS node.
            topic_name (str): Name of the topic where orders are published.
            order_queue (collections.deque): Queue to store orders.
            callback_group: Callback group for this class.
        '''
        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )
        self.node = node
        self.order_topic1 = topic_name1
        self._orders = order_queue
        self.orders_subcriber = node.create_subscription(OrderMsg, self.order_topic1, self._orders_callback, 10, callback_group=callback_group)
        self._parsing_Flag = True
        node.set_parameters([sim_time])
        # Subscriber to the logical camera topic


    @property
    def orders(self):
        '''
        Property to access the orders queue.

        Returns:
            collections.deque: The queue containing orders.
        '''
        return self._orders

    @property
    def _the_order_to_parse(self):
        '''
        Property to get the parsing flag.

        Returns:
            bool: The parsing flag indicating if the order should be parsed.
        '''
        return self._parsing_Flag
    
    @_the_order_to_parse.setter
    def _the_order_to_parse(self, value=True):
        '''
        Setter for the parsing flag.

        Args:
            value (bool): The value to set for the parsing flag.
        '''
        self._parsing_Flag = value
        
    def _orders_callback(self, msg: Order):
        '''
        Callback function for order messages.

        Args:
            msg (Order): The received order message.
        '''
        order = Order(msg)

        if order.order_priority:
            self._orders.appendleft(order)
        else:
            self._orders.append(order)

        if self._parsing_Flag:
            self.node.get_logger().info(self._parse_the_order(order))
   
    def _parse_the_order(self, order: Order):
        '''
        Parse an order message and return a string representation.

        Args:
            order (Order): The order message.

        Returns:
            str: String representation of the order message.
        '''
        output = '\n\n==========================\n'
        output += f'Order ID of Received Order: {order.order_id}\n'
        output += f'Priority of the Order : {order.order_priority}\n'

        if order.order_type == OrderMsg.KITTING:
            output += self._parse_kitting_task(order.order_task)
        else:
            output += 'Type: Unknown\n'

        return output

