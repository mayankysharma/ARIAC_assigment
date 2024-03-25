from ariac_msgs.msg import AGVStatus  # Import the AGVStatus message type
from ariac_msgs.srv import SubmitOrder  # Import the SubmitOrder service
from ariac_msgs.msg import Order  # Import the Order message type

class OrderSubmission():
    def __init__(self, node, service_name):
        self.subscribers = {}
        self.node = node
        self.service_name = service_name
        self.submit_order_client = self.node.create_client(SubmitOrder, service_name)  # Create a client for the SubmitOrder service

    def Submit_Order(self, agv_id, order_id):
        self.order_id = order_id
        self.agv_id = agv_id
        if self.agv_id not in self.subscribers:
            # Create a subscriber for the AGV status topic
            self.subscribers[self.agv_id]  = self.node.create_subscription(AGVStatus, '/ariac/agv{}_status'.format(self.agv_id), self.agv_status_callback)
            # self.subscribers[self.agv_id] = self.agv_status_subscriber
        # else:
        #     # If a subscriber already exists, reuse it
        #     self.agv_status_subscriber = self.subscribers[self.agv_id]

    def agv_status_callback(self, msg):
        # Check if the AGV has reached the warehouse
        if msg.location == AGVStatus.WAREHOUSE:
            # Submit the order
            self.call_submit_order_service()

    def call_submit_order_service(self):
        request = SubmitOrder.Request()
        request.order_id = self.order_id
        # Call the SubmitOrder service
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
