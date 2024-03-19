import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
    Order as OrderMsg,
    Part as PartMsg,

    PartPose as PartPoseMsg,
    BreakBeamStatus as BreakBeamStatusMsg,

    AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,

    AssemblyPart as AssemblyPartMsg,

    AGVStatus as AGVStatusMsg,

    AssemblyTask as AssemblyTaskMsg,

)
from rwa3_group2.utils import (

    multiply_pose,

    rpy_from_quaternion,

    rad_to_deg_str,

    AdvancedLogicalCameraImage,

    Order,

    KittingTask,
    KittingPart,

)

from std_srvs.srv import Trigger


class CompetitionInterface(Node):
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

    '''Dictionary for converting Part color constants to emojis'''


    _part_types = {

        PartMsg.BATTERY: 'battery',

        PartMsg.PUMP: 'pump',

        PartMsg.REGULATOR: 'regulator',

        PartMsg.SENSOR: 'sensor',

    }

    '''Dictionary for converting Part type constants to strings'''


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

    '''Dictionary for converting AssemblyTask constants to strings'''

    

    def __init__(self):
        super().__init__('competition_interface')

        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )

        self.set_parameters([sim_time])
        # Service client for starting the competition
        self._start_competition_client = self.create_client(
            Trigger, '/ariac/start_competition')

        # Subscriber to the competition state topic
        self._competition_state_sub = self.create_subscription(
            CompetitionStateMsg,
            '/ariac/competition_state',
            self._competition_state_cb,
            10)

        # Store the state of the competition
        self._competition_state: CompetitionStateMsg = None
        self._conveyor_part_count = 0
        self.orders_sub = self.create_subscription(

            OrderMsg,

            '/ariac/orders',

            self._orders_cb,

            10)


        # Flag for parsing incoming orders
        self._camera_image: AdvancedLogicalCameraImage = None
        self._parse_incoming_order = False


        # List of orders

        self._orders = []
    
    @property

    def orders(self):

        return self._orders


    @property

    def camera_image(self):

        return self._camera_image


    @property

    def conveyor_part_count(self):

        return self._conveyor_part_count


    @property

    def parse_incoming_order(self):

        return self._parse_incoming_order


    @parse_incoming_order.setter

    def parse_incoming_order(self, value):

        self._parse_incoming_order = value

    def _advanced_camera0_cb(self, msg: AdvancedLogicalCameraImageMsg):

        '''Callback for the topic /ariac/sensors/advanced_camera_0/image


        Arguments:

            msg -- AdvancedLogicalCameraImage message

        '''

        self._camera_image = AdvancedLogicalCameraImage(msg.part_poses,

                                                        msg.tray_poses,

                                                        msg.sensor_pose)


    def _breakbeam0_cb(self, msg: BreakBeamStatusMsg):

        '''Callback for the topic /ariac/sensors/breakbeam_0/status


        Arguments:

            msg -- BreakBeamStatusMsg message

        '''

        if not self._object_detected and msg.object_detected:

            self._conveyor_part_count += 1


        self._object_detected = msg.object_detected
    def _orders_cb(self, msg: Order):

        '''Callback for the topic /ariac/orders

        Arguments:

            msg -- Order message

        '''

        order = Order(msg)

        self._orders.append(order)

        if self._parse_incoming_order:

            self.get_logger().info(self._parse_order(order))
            
    
    def _competition_state_cb(self, msg: CompetitionStateMsg):
        '''Callback for the topic /ariac/competition_state

        Arguments:
            msg -- CompetitionState message
        '''
        # Log if competition state has changed
        if self._competition_state != msg.competition_state:
            state = CompetitionInterface._competition_states[msg.competition_state]
            self.get_logger().info(f'Competition state is: {state}', throttle_duration_sec=1.0)

        self._competition_state = msg.competition_state
        

    def start_competition(self):
        '''Function to start the competition.
        '''
        self.get_logger().info('Waiting for competition to be ready')

        if self._competition_state == CompetitionStateMsg.STARTED:
            return
        # Wait for competition to be ready
        while self._competition_state != CompetitionStateMsg.READY:
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                return

        self.get_logger().info('Competition is ready. Starting...')

        # Check if service is available
        if not self._start_competition_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Service \'/ariac/start_competition\' is not available.')
            return

        # Create trigger request and call starter service
        request = Trigger.Request()
        future = self._start_competition_client.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Started competition.')
        else:
            self.get_logger().warn('Unable to start competition')
            
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

        output += f'Destination: {CompetitionInterface._destinations[kitting_task.destination]}\n'

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

                    part_color = CompetitionInterface._part_colors[product.part.color].capitalize()

                    part_color_emoji = CompetitionInterface._part_colors_emoji[product.part.color]

                    part_type = CompetitionInterface._part_types[product.part.type].capitalize()

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

        elif order.order_type == OrderMsg.ASSEMBLY:

            output += self._parse_assembly_task(order.order_task)

        elif order.order_type == OrderMsg.COMBINED:

            output += self._parse_combined_task(order.order_task)

        else:

            output += 'Type: Unknown\n'

        return output
