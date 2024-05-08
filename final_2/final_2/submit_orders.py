from ariac_msgs.msg import AGVStatus  # Import the AGVStatus message type
from ariac_msgs.srv import SubmitOrder  # Import the SubmitOrder service
from ariac_msgs.msg import Order  # Import the Order message type
from ariac_msgs.srv import MoveAGV
from ariac_msgs.msg import KittingTask as KittingTaskMsg

from functools import partial

class ShipAndOrderSubmission():
    ''' 
    Class to handle order submission to AGVs.
    '''

    def __init__(self, node, service_name, callback_group, submit_callback_group):
        ''' 
        Initialize the OrderSubmission object.

        Args:
            node: The ROS node.
            service_name (str): The name of the SubmitOrder service.
            callback_group: The callback group.
        '''
        self.subscribers = {}  # Initialize subscribers dictionary to store AGV status subscribers
        self.node = node  # Store the ROS node
        self.service_name = service_name  # Store the name of the SubmitOrder service
        self.submit_order_client = self.node.create_client(SubmitOrder, service_name, callback_group=submit_callback_group)  # Create a client for the SubmitOrder service with a specified callback group
        self.callback_group = callback_group  # Store the callback group
        self.agv_loc = {}  # Initialize dictionary to store AGV locations
        # Create subscribers for AGV status messages for each AGV
        for i in range(1, 5):
            self.subscribers[i] = self.node.create_subscription(AGVStatus, 
                                                                '/ariac/agv{}_status'.format(i), 
                                                                partial(self.agv_status_callback, agv_num=i),
                                                                10,
                                                                callback_group=self.callback_group)

        self.agv_lock = {}  # Dictionary to store AGV lock clients
        self.agv_move = {}  # Dictionary to store AGV move clients

    def Submit_Order(self, agv_num, order_id):
        ''' 
        Submit an order to a specified AGV.

        Args:
            agv_num (int): The number of the AGV.
            order_id (str): The ID of the order.

        Returns:
            bool: True if order submission is successful, False otherwise.
        '''
        self.order_id = order_id  # Store the order ID
        self.agv_num = agv_num  # Store the AGV number
        
        # Check if the AGV is at the warehouse
        if self.agv_loc[self.agv_num] == AGVStatus.WAREHOUSE:
            # If AGV is at the warehouse, submit the order
            self.call_submit_order_service()
            return True  # Return True indicating successful order submission
        return False  # Return False if AGV is not at the warehouse
        
    def agv_status_callback(self, msg, agv_num):
        ''' 
        Callback function to update AGV locations.

        Args:
            msg: The AGVStatus message.
            agv_num (int): The number of the AGV.
        '''
        self.agv_loc[agv_num] = msg.location 

    def call_submit_order_service(self):
        ''' 
        Call the SubmitOrder service to submit the order.
        '''
        # Create a request object for the SubmitOrder service
        request = SubmitOrder.Request()
        request.order_id = self.order_id  # Set the order ID in the request
        # Call the SubmitOrder service
        future = self.submit_order_client.call(request)
        # Handle the response asynchronously
        self.order_submission_callback(future)

    def order_submission_callback(self, future):
        ''' 
        Callback function to handle the response from the SubmitOrder service.

        Args:
            future: The future object representing the response from the service call.
        '''
        try:
            # Retrieve the response from the future object
            response = future
            if response.success:
                # If order submission is successful, log a success message
                self.node.get_logger().info("Order submitted successfully.")
                self.node._order_submitted = True
                return  # Return True if order submission is successful
            else:
                # If order submission fails, log an error message
                self.node.get_logger().error("Order submission failed: {}".format(response.message))
                return  # Return False if order submission fails
        except Exception as e:
            # Log an error message if service call fails
            self.node.get_logger().error("Service call failed: {}".format(e))
            return  # Return False if service call fails


    def move_agv_to_station(self,future, agv_number,order_id):
        '''
        Move an AGV to an assembly station.

        Args:
            num (int): AGV number
            station (int): Assembly station number

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''

        # Check the result of the service call.
        if future.result().success:
            self.node.get_logger().info(f'Moved AGV to Warehouse')
            
            # Wait till the order is not submit
            while not self.Submit_Order(agv_num=agv_number,order_id=order_id): continue
        else:
            self.node.get_logger().warn(future.result().message)
            raise Exception("Unable to Move") 
    
    def lock_move_agv(self, order):
        ''' 
        Lock the tray of an AGV and move it to the specified destination for shipping.

        Args:
            order (OrderMsg): The order message.

        Returns:
            Tuple[str, int] or None: A tuple containing the order ID and AGV number if successful, otherwise None.
        '''
        try:
            # Retrieve the tray id
            tray_num = order.order_task.tray_id
            # Retrieve the agv number
            agv_num = order.order_task.agv_number
            # Retrieve the destination
            ship_destination = order.order_task.destination

            # Create a client to send a request to the `/ariac/move_agv` service.
            if agv_num not in self.agv_move:
                self.agv_move[agv_num] = self.node.create_client(MoveAGV,f'/ariac/move_agv{agv_num}',callback_group=self.callback_group)

            # Create a request object.
            request = MoveAGV.Request()

            # Set the request location.
            if ship_destination == KittingTaskMsg.WAREHOUSE:
                request.location = MoveAGV.Request.WAREHOUSE

            self.node.get_logger().info(f'AGV num {agv_num} and order id {order.order_id}')
            
            # Send the request.
            future = self.agv_move[agv_num].call_async(request)
            future.add_done_callback(partial(self.move_agv_to_station,agv_number=agv_num,order_id=order.order_id))
            # self.agv_tray_locked(agv_num)

            return (order.order_id, agv_num)
        except Exception as e:
            # print(e)
            self.node.get_logger().error(f"{e}")
            return None
