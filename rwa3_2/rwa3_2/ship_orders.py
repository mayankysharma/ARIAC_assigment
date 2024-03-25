import rclpy
from std_srvs.srv import Trigger
from ariac_msgs.srv import MoveAGV
from ariac_msgs.msg import KittingTask as KittingTaskMsg
from ariac_msgs.msg import Order as OrderMsg

class ShipOrders():
    def __init__(self, node):
        self.node = node
        self.agv_lock = {}
        self.agv_move = {}

    def agv_tray_locked(self, num):
        '''
        Lock the tray of an AGV and parts on the tray. This will prevent tray and parts from moving during transport.

        Args:
            num (int):  AGV number

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''

        # Create a client to send a request to the `/ariac/agv{num}_lock_tray` service
        # create service for specific agv_ID
        if num not in self.agv_lock:
            self.agv_lock[num] = self.node.create_client(Trigger,f'/ariac/agv{num}_lock_tray')

        # Build the request
        request = Trigger.Request()
        # Send the request
        future = self.agv_lock[num].call(request)

        # Wait for the response
        # try:
        #     rclpy.spin_until_future_complete(self, future)
        # except KeyboardInterrupt as kb_error:
        #     raise KeyboardInterrupt from kb_error

        # Check the response
        if future.success:
            self.node.get_logger().info(f'AGV{num}\'s tray locked')
        else:
            self.node.get_logger().warn('Unable to lock tray')

    def move_agv_to_station(self, num, station):
        '''
        Move an AGV to an assembly station.

        Args:
            num (int): AGV number
            station (int): Assembly station number

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''

        # Create a client to send a request to the `/ariac/move_agv` service.
        if num not in self.agv_move:
            self.agv_move[num] = self.node.create_client(MoveAGV,f'/ariac/move_agv{num}')

        # Create a request object.
        request = MoveAGV.Request()

        # Set the request location.
        if station == KittingTaskMsg.WAREHOUSE:
            request.location = MoveAGV.Request.WAREHOUSE

        # Send the request.
        future = self.agv_move[num].call(request)

        # Wait for the server to respond.
        # try:
        #     rclpy.spin_until_future_complete(self, future)
        # except KeyboardInterrupt as kb_error:
        #     raise KeyboardInterrupt from kb_error

        # Check the result of the service call.
        if future.success:
            self.node.get_logger().info(f'Moved AGV{num} to Warehouse')
        else:
            self.node.get_logger().warn(future.message)
    
    def lock_move_agv(self):
        # Retrieve the tray id
        self.tray_num = OrderMsg.tray_id
        # Retrieve the agv number
        self.agv_num = OrderMsg.agv_number
        # Retrieve the destination
        self.ship_destination = OrderMsg.destination
        self.agv_tray_locked(self.tray_num)
        self.move_agv_to_station(self.agv_num, self.ship_destination)

    
