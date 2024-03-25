
from ariac_msgs.msg import AGVStatus  # Import the AGVStatus message type
from ariac_msgs.srv import SubmitOrder  # Import the SubmitOrder service
from ariac_msgs.msg import Order  # Import the Order message type
from functools import partial

class OrderSubmission():
    def __init__(self, node, service_name, callback_group):
        # Initialize subscribers dictionary to store AGV status subscribers
        self.subscribers = {}
        self.node = node  # Store the ROS node
        self.service_name = service_name  # Store the name of the SubmitOrder service
        # Create a client for the SubmitOrder service with a specified callback group
        self.submit_order_client = self.node.create_client(SubmitOrder, service_name, callback_group=callback_group)  
        self.callback_group = callback_group  # Store the callback group
        self.agv_loc = {}  # Initialize dictionary to store AGV locations
        # Create subscribers for AGV status messages for each AGV
        for i in range(1, 5):
            self.subscribers[i] = self.node.create_subscription(AGVStatus, 
                                                                '/ariac/agv{}_status'.format(i), 
                                                                partial(self.agv_status_callback, agv_num=i),
                                                                10,
                                                                callback_group=self.callback_group)
        
    def Submit_Order(self, agv_num, order_id):
        self.order_id = order_id  # Store the order ID
        self.agv_num = agv_num  # Store the AGV number
        
        # Check if the AGV is at the warehouse
        if self.agv_loc[self.agv_num] == AGVStatus.WAREHOUSE:
            # If AGV is at the warehouse, submit the order
            self.call_submit_order_service()
            return True  # Return True indicating successful order submission
        return False  # Return False if AGV is not at the warehouse
        
    def agv_status_callback(self, msg, agv_num):
        # Callback function to update AGV locations
        self.agv_loc[agv_num] = msg.location 

    def call_submit_order_service(self):
        # Create a request object for the SubmitOrder service
        request = SubmitOrder.Request()
        request.order_id = self.order_id  # Set the order ID in the request
        # Call the SubmitOrder service
        future = self.submit_order_client.call(request)
        # Handle the response asynchronously
        self.order_submission_callback(future)

    def order_submission_callback(self, future):
        try:
            # Retrieve the response from the future object
            response = future
            if response.success:
                # If order submission is successful, log a success message
                self.node.get_logger().info("Order submitted successfully.")
                return  # Return True if order submission is successful
            else:
                # If order submission fails, log an error message
                self.node.get_logger().error("Order submission failed: {}".format(response.message))
                return  # Return False if order submission fails
        except Exception as e:
            # Log an error message if service call fails
            self.node.get_logger().error("Service call failed: {}".format(e))
            return  # Return False if service call fails
