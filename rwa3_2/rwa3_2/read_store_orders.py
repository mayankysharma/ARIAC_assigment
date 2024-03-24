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
    Order,
  

    KittingTask,

    KittingPart
)

class ReadStoreOrders():
    _destinations = {

        AGVStatusMsg.KITTING: 'kitting station',

        AGVStatusMsg.ASSEMBLY_FRONT: 'front assembly station',

        AGVStatusMsg.ASSEMBLY_BACK: 'back assembly station',

        AGVStatusMsg.WAREHOUSE: 'warehouse',

        }

    '''Dictionary for converting AGVDestination constants to strings'''


    _stations = {

        AssemblyTaskMsg.AS1: 'assembly station 1',

        AssemblyTaskMsg.AS2: 'assembly station 2',

        AssemblyTaskMsg.AS3: 'assembly station 3',

        AssemblyTaskMsg.AS4: 'assembly station 4',

        }
    _part_colors = {

        PartMsg.RED: 'red',

        PartMsg.BLUE: 'blue',

        PartMsg.GREEN: 'green',

        PartMsg.ORANGE: 'orange',

        PartMsg.PURPLE: 'purple',

    }
    '''Dictionary for converting Part color constants to strings'''


    _part_colors_emoji = {

        PartMsg.RED: '🟥',

        PartMsg.BLUE: '🟦',

        PartMsg.GREEN: '🟩',

        PartMsg.ORANGE: '🟧',

        PartMsg.PURPLE: '🟪',

    }
    _part_types = {

        PartMsg.BATTERY: 'battery',

        PartMsg.PUMP: 'pump',

        PartMsg.REGULATOR: 'regulator',

        PartMsg.SENSOR: 'sensor',

    }

    '''Dictionary for converting AssemblyTask constants to strings'''

    def __init__(self, node, topic_name, order_queue,callback_group):
        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )
        self.node = node
        #topic to which the orders are published
        self.order_topic=topic_name
        #List for now for the orders
        self._orders = order_queue
        # Subscriber to the order topic

        self.orders_sub = node.create_subscription(OrderMsg, self.order_topic, self._orders_cb,10,callback_group = callback_group)


        # Flag for parsing incoming orders

        self._parse_incoming_order = True
        node.set_parameters([sim_time])

        # List of orders

        # self._orders = []
    @property

    def orders(self):

        return self._orders
    @property

    def parse_incoming_order(self):

        return self._parse_incoming_order
    @parse_incoming_order.setter

    def parse_incoming_order(self, value=True):

        self._parse_incoming_order = value
        
    def _orders_cb(self, msg: Order):

        '''Callback for the topic /ariac/orders

        Arguments:

            msg -- Order message

        '''

        order = Order(msg)

        self._orders.append(order)

        if self._parse_incoming_order:
            print("Here")
            self.node.get_logger().info(self._parse_order(order))
            
    def _parse_kitting_task(self, kitting_task: KittingTask):

        '''

        Parses a KittingTask object and returns a string representation.


        Args:

            kitting_task (KittingTask): KittingTask object to parse


        Returns:

            str: String representation of the KittingTask object

        '''

        output = 'Type: Kitting\n'

        output += '==========================\n'

        output += f'AGV: {kitting_task.agv_number}\n'

        output += f'Destination: {ReadStoreOrders._destinations[kitting_task.destination]}\n'

        output += f'Tray ID: {kitting_task.tray_id}\n'

        output += 'Products:\n'

        output += '==========================\n'


        quadrants = {1: "Quadrant 1: -",

                    2: "Quadrant 2: -",

                    3: "Quadrant 3: -",

                    4: "Quadrant 4: -"}


        for i in range(1, 5):

            product: KittingPart

            for product in kitting_task.parts:

                if i == product.quadrant:

                    part_color = ReadStoreOrders._part_colors[product.part.color].capitalize()

                    part_color_emoji = ReadStoreOrders._part_colors_emoji[product.part.color]

                    part_type = ReadStoreOrders._part_types[product.part.type].capitalize()

                    quadrants[i] = f'Quadrant {i}: {part_color_emoji} {part_color} {part_type}'

        output += f'\t{quadrants[1]}\n'

        output += f'\t{quadrants[2]}\n'

        output += f'\t{quadrants[3]}\n'

        output += f'\t{quadrants[4]}\n'


        return output
    def _parse_order(self, order: Order):

        '''Parse an order message and return a string representation.


        Args:

            order (Order) -- Order message


        Returns:

            String representation of the order message

        '''

        output = '\n\n==========================\n'

        output += f'Received Order: {order.order_id}\n'

        output += f'Priority: {order.order_priority}\n'


        if order.order_type == OrderMsg.KITTING:

            output += self._parse_kitting_task(order.order_task)
        else:

            output += 'Type: Unknown\n'

        return output
        

