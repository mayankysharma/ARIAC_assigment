from ariac_msgs.msg import AGVStatus  # Import the AGVStatus message type
from ariac_msgs.srv import SubmitOrder  # Import the SubmitOrder service
from ariac_msgs.msg import Order  # Import the Order message type
from functools import partial

class OrderSubmission():
    def __init__(self, node, service_name, callback_group):
        self.subscribers = {}
        self.node = node
        self.service_name = service_name
        self.submit_order_client = self.node.create_client(SubmitOrder, service_name, callback_group = callback_group)  # Create a client for the SubmitOrder service
        self.callback_group = callback_group
        self.agv_loc = {}
        for i in range(1,5):
            self.subscribers[i]  = self.node.create_subscription(AGVStatus, 
                                                                        '/ariac/agv{}_status'.format(i), 
                                                                        partial(self.agv_status_callback,agv_num=i),
                                                                        10,
                                                                        callback_group = self.callback_group)
    def Submit_Order(self, agv_num, order_id):
        self.order_id = order_id
        self.agv_num = agv_num
        
        # self.node.get_logger().info(f"{self.agv_loc[self.agv_num]}, {agv_num}")
        if self.agv_loc[self.agv_num]==AGVStatus.WAREHOUSE:
        # print(self.agv_loc[self.agv_num])
            self.call_submit_order_service()
            return True
        return False
        
    def agv_status_callback(self, msg, agv_num):
        # Check if the AGV has reached the warehouse
        # print(msg.location)
        # if msg.location == AGVStatus.WAREHOUSE:
            # Submit the order
        self.agv_loc[agv_num] = msg.location 

    def call_submit_order_service(self):
        request = SubmitOrder.Request()
        request.order_id = self.order_id
        # Call the SubmitOrder service
        # print("send the submission request")
        future = self.submit_order_client.call(request)
        self.order_submission_callback(future)

    def order_submission_callback(self, future):
        try:
            response = future
            if response.success:
                self.node.get_logger().info("Order submitted successfully.")
                return  # Return True if order submission is successful
            else:
                self.node.get_logger().error("Order submission failed: {}".format(response.message))
                return  # Return False if order submission fails
        except Exception as e:
            self.node.get_logger().error("Service call failed: {}".format(e))
            return # Return False if service call fails
