
import time
from collections import deque
from copy import deepcopy


from rclpy.node import Node
import rclpy 
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# from ship_orders import ShipOrders
from custom_timer import CustomTimer
from comp_state import CompetitionState
from read_store_orders import ReadStoreOrders
from ship_orders import ShipOrders

from submit_orders import OrderSubmission  
from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
    AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
    Part as PartMsg,
    PartPose as PartPoseMsg,
)
from sensor_read import SensorRead
class AriacInterface(Node):
    """
    Class representing the interface for managing ARIAC competition tasks.
    """
    
    order_topic1 = "/ariac/orders"
    sensor_topic1 = "/ariac/sensors/kts1_camera/image"
    
    comp_start_state_service_name = "/ariac/start_competition"
    comp_end_state_service_name = "/ariac/end_competition"
    comp_state_topic_name = "/ariac/competition_state"
    submit_order_service_name = "/ariac/submit_order"

    def __init__(self, node_name):
        """
        Initialize the AriacInterface node.

        :param node_name: Name of the ROS node.
        """
        super().__init__(node_name)
        group_mutex1 = MutuallyExclusiveCallbackGroup()
        group_reentrant1 = ReentrantCallbackGroup()
        self.order_queue = deque()

         #Competition State object instance
        self.comp_state = CompetitionState(self, AriacInterface.comp_state_topic_name, AriacInterface.comp_start_state_service_name, AriacInterface.comp_end_state_service_name, callback_group=group_reentrant1)
        self._monitor_state = self.create_timer(1, self.monitor_state_callback)
        self.order_submit = OrderSubmission(self,AriacInterface.submit_order_service_name,group_reentrant1)

        self.ship_order = ShipOrders(self,group_reentrant1)
        #Read and store order object instance
        self.read_store_orders=ReadStoreOrders(self,AriacInterface.order_topic1,self.order_queue,callback_group=group_reentrant1)
        self.sensor_read=SensorRead(self,callback_group=group_reentrant1)
        
        # self.custom_timer_t0 = CustomTimer(self)
        # self.custom_timer_t1 = CustomTimer(self)
        self.current_order_priority = False
        self.current_order = None
        self.pending_order = None

    def monitor_state_callback(self):
        """
        Callback function to monitor the state of the competition.
        """
        if self.comp_state.competition_started and not self.comp_state.competition_ended:
            self.fulfill_orders()

    def fulfill_orders(self):
        """
        Method to fulfill orders during the competition.
        """
        # self.get_logger().info("Done waiting, taking order now, Delay 15 secs for new order!!")

        if len(self.order_queue)>0:
            try:
                order = self.order_queue.popleft()
                # if len(self.current_order)==0:
                if order.order_priority and not self.current_order_priority:
                    self.current_order_priority = order.order_priority
                    self.pending_order = self.current_order
                    self.pending_order[1].pause()
                    self.get_logger().info(f"Keeping the {self.pending_order[0].order_id} to pending!!")
                    self.current_order = (order,CustomTimer(self,"t1"),order.order_priority)
                    self.get_logger().info(f"Got the High Priority Order Changing to it of id {self.current_order[0].order_id}!!")
                        # return
                elif self.current_order is None and self.pending_order is None:
                    self.current_order = (order,CustomTimer(self,"t0"),self.current_order_priority)
                else:
                    self.order_queue.appendleft(order)
            except Exception as e:
                self.get_logger().warn(f"Unable to take order because of {e}!")

        if self.current_order is not None:
            order, t, curr_priority = self.current_order
            try:
                # s = self.node.get_clock().now()
                # if t.check_delay_flag():
                #     return
                # end_t = self.node.get_clock().now()
                """
                Example print for sensor information
                - KITTING01
                    - Kitting Tray:
                        - ID: 1
                        - Position (xyz): [-0.870000, -5.840000, 0.734989]
                        - Orientation (rpy): [0.0, 0.0, 3.14]
                    - Orange Battery:
                        - Position (xyz): [-2.080000, 2.445000, 0.719998]
                        - Orientation (rpy): [0.0, 0.0, 3.14]
                    - Green Sensor:
                        - Position (xyz): [-2.080000, 2.805000, 0.719998]
                        - Orientation (rpy): [0.0, 0.0, 3.14
                """
                self.get_logger().info("Got the Order!!")
                for sensor_name, sensor_data in self.sensor_read.sensor_data.items():
                    for sdata in sensor_data:
                        if sdata["is_part"]:
                            for pdata in order.order_task.parts:
                                if sdata["type"]==pdata.type and sdata["color"] == pdata.color:
                                    self.get_logger.info(f" Here is the sensor data: \n {sdata}")
                        else:
                            for tid in order.order_task.tray_id:
                                if sdata["tray_id"]==tid:
                                    self.get_logger.info(f" Here is the sensor data: \n {sdata}")
                self.get_logger().info(f"Starting the shipping and submitting the order {order.order_id}!!!")

                data = self.ship_order.lock_move_agv(order)
                if data is None:
                    self.get_logger().warn("Unable to ship!")

                while not self.order_submit.Submit_Order(agv_num=data[1],order_id=data[0]): continue
                self.current_order = self.pending_order
                if self.pending_order is not None:
                    self.current_order_priority = self.pending_order[2]
                self.pending_order = None
            except Exception as e:
                self.get_logger().warn(f"Unable to ship and submit the order because of {e}!")
        else:
            if self.comp_state.all_orders_recieved:
                self.comp_state.competition_ended = True

        # self.custom_timer.reset_flags()

