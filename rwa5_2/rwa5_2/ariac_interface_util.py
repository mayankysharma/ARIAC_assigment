
import time
from collections import deque
from copy import deepcopy

from rclpy.node import Node
import rclpy 
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import Pose
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

# from ship_orders import ShipOrders
from custom_timer import CustomTimer
from comp_state import CompetitionState
from read_store_orders import ReadStoreOrders
from ship_orders import ShipOrders
from utils import RPY_to_Quart

from submit_orders import OrderSubmission  
from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
    AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
    Part as PartMsg,
    PartPose as PartPoseMsg,
)
from std_srvs.srv import Trigger
from rwa5_2.srv import PickPlace


from process_order import ProcessOrder
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

    pause_service_name = "/robot/pause"
    resume_service_name = "/robot/resume"
    move_service_name="/robot/move"

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
        
        # pause service use for the pausing the robot when recived high priority order
        self.pause_service = self.create_client(Trigger,
                                                AriacInterface.pause_service_name,
                                                callback_group=group_reentrant1)

        # Resume service for the previous order
        self.resume_service = self.create_client(Trigger,
                                                AriacInterface.resume_service_name,
                                                callback_group=group_reentrant1)
        # Resume service for the previous order
        self.move_service = self.create_client(PickPlace,
                                                AriacInterface.move_service_name,
                                                callback_group=group_reentrant1)

        self.current_order_priority = False
        self.current_order = None
        self.pending_order = None

        # Transform listener to get the pose of some frames not predefined
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        if len(self.order_queue)>0:
            try:
                order = self.order_queue.popleft()
                # if len(self.current_order)==0:
                self.update_order(order)
            except Exception as e:
                self.get_logger().warn(f"Unable to take order because of {e}!")

        if self.current_order is not None:
            order, process_order, curr_priority = self.current_order

            if not process_order.recievedOrder:
                # if not recieved the order details from sensor, then fetch it.
                try:
                    tray_info, parts_info = self.get_info_from_sensor(order,verbose=False)
                    # self.get_logger().info(f"tray info from sensor : {tray_info}")
                    # self.get_logger().info(f"part info from sensor : {parts_info}")
                    process_order.getOrder(tray_info=tray_info, parts_info=parts_info)
                except Exception as e:
                    self.get_logger().error(f"PROBLEM WITH THE GETING THE PARTS AND TRAY INFO FROM ENVIRONMENT!!!! \n {e}")
                    
            if not process_order.isOrderProcessed:
                self.get_logger().info(f"Processing the order {order.order_id}!!")

                # Start processing the order
                process_order.get_pick_place_position()
                    
            else:              
                try:
                    # Start shipping and submitting the order
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


    def update_order(self, order):
        """
        update the current order with high priority order, or assign the new order if not present.

        Args:
            order: order class contain info of all order details
        """
        if order.order_priority and not self.current_order_priority:

            if self.current_order is not None:
                # pausing the current processing of the order if there is any.
                try:
                    self.current_order[1].pause()
                except Exception as e:
                    self.get_logger().fatal(f"Wait for Previous Order to complete causing error {e}")
                    self.order_queue.appendleft(order)
            
            self.current_order_priority = order.order_priority
            self.pending_order = self.current_order

            self.current_order = (order, ProcessOrder(order_id=order.order_id,node=self), order.order_priority)
            self.get_logger().info(f"Got the High Priority Order!! Changing to it of id {self.current_order[0].order_id}!!")

        elif self.current_order is None and self.pending_order is None:
            self.current_order = (order, ProcessOrder(order_id=order.order_id,node=self), self.current_order_priority)
            self.get_logger().info(f"Got the Order!! Order Id : {order.order_id}")
        else:
            self.order_queue.appendleft(order)

    def get_info_from_sensor(self, order, verbose = False) -> tuple:
        """
        Retreive the order part info from sensor data and which will be use to pass it to robot for further processing

        and will print the order part and tray info as well, if verbose = True
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
                - Orientation (rpy): [0.0, 0.0, 3.14]
        Args:
            order : order class, contains order details
        
        Return:
            tuple :
                tray_info : dict
                parts_info : list(dict)
        """

         # store the tray information
        print_tray = {}
        tray_info = {}
        # key : tray_id, value : [pick pose, place pose]
        
        # store the part information
        print_parts = {}
        parts_info = []
        # key : (type, color, quadrant, tray_id)

        visited_data = []
        for sensor_name, sensor_data in self.sensor_read.sensor_data.items():
            for sdata in sensor_data:
                if sdata["is_part"]:
                    for pdata in order.order_task.parts:
                        if (pdata.part.type, pdata.part.color, pdata.quadrant) in print_parts.keys():
                            continue
                        if sdata["type"]==pdata.part.type and sdata["color"] == pdata.part.color and sdata["pose"] not in visited_data:
                            visited_data.append(sdata["pose"])

                            # Store the pose, tray_id and agv_num for processing the tray like pick and place
                            pose = Pose()
                            pose.position.x,pose.position.y, pose.position.z = sdata["pose"]
                            quart = RPY_to_Quart(sdata["orientation"])
                            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) = quart 

                            parts_info.append(
                                {
                                    "type" : pdata.part.type,
                                    "quadrant" : pdata.quadrant,
                                    "color" : sdata["color"],
                                    "pose" : pose,
                                    "agv_num" : order.order_task.agv_number,
                                    "pick" : True,
                                    "kts" : 2 if order.order_task.agv_number<3 else 1 # Reason being agv1 and 2 are near to kts2
                                }
                            )

                            print_parts[(sdata["type"], sdata["color"], pdata.quadrant)] = f"""
            - {ReadStoreOrders._color_of_parts[pdata.part.color]} {ReadStoreOrders._type_of_parts[pdata.part.type]}
                - Position (xyz): [{sdata["pose"][0]:.3f}, {sdata["pose"][1]:.3f}, {sdata["pose"][2]:.3f}]
                - Orientation (rpy): [{sdata["orientation"][0]:.3f}, {sdata["orientation"][1]:.3f}, {sdata["orientation"][2]:.3f}]"""
                else:
                    if sdata["tray_id"]==order.order_task.tray_id:

                        # Store the pose, tray_id and agv_num for processing the tray like pick and place
                        pose = Pose()
                        pose.position.x,pose.position.y, pose.position.z = sdata["pose"]
                        quart = RPY_to_Quart(sdata["orientation"])
                        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) = quart 
                        tray_info = {
                            "id" : order.order_task.tray_id,
                            "pose" : pose,
                            "agv_num" : order.order_task.agv_number,
                            "pick" : True,
                            "kts" : 1 if "kts1" in sensor_name else 2
                        }


                        ## Prints
                        print_tray["tray_id"] = f"""
                - ID : {order.order_task.tray_id}
                - Position (xyz): [{sdata["pose"][0]:.3f}, {sdata["pose"][1]:.3f}, {sdata["pose"][2]:.3f}]
                - Orientation (rpy): [{sdata["orientation"][0]:.3f}, {sdata["orientation"][1]:.3f}, {sdata["orientation"][2]:.3f}]"""

        if verbose:
            ## printing the part and tray info
            order_details_print = f"""\n==========================
            - {order.order_id}
                - Kitting Tray"""
            # print(parts)
            for tray_id, ptray_info in print_tray.items():
                order_details_print += ptray_info
            for part_, part_details in print_parts.items():
                order_details_print += part_details
            order_details_print += "\n==========================\n"
            self.get_logger().info(order_details_print)
        return tray_info, parts_info