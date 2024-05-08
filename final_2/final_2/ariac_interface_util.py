
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

from custom_timer import CustomTimer
from comp_state import CompetitionState
from read_store_orders import ReadStoreOrders
from utils import RPY_to_Quart

# from submit_orders import OrderSubmission  
from submit_orders import ShipAndOrderSubmission

from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
    AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
    Part as PartMsg,
    PartPose as PartPoseMsg,
    VacuumGripperState as VacuumGripperStateMsg
)
from std_srvs.srv import Trigger
from ariac_msgs.srv import ChangeGripper, VacuumGripperControl

# Import custom ROS services
from robot_commander_msgs.srv import (
    EnterToolChanger,
    ExitToolChanger,
    MoveRobotToTable,
    MoveRobotToTray,
    MoveTrayToAGV,
    PickPart,
    PlacePart
)

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


    def __init__(self, node_name):
        """
        Initialize the AriacInterface node.

        :param node_name: Name of the ROS node.
        """
        super().__init__(node_name)
        group_mutex1 = MutuallyExclusiveCallbackGroup()
        group_reentrant1 = ReentrantCallbackGroup()
        robot_cbg = ReentrantCallbackGroup()
        ship_cbg = ReentrantCallbackGroup()
        self.order_queue = deque()

         #Competition State object instance
        self.comp_state = CompetitionState(self, AriacInterface.comp_state_topic_name, AriacInterface.comp_start_state_service_name, AriacInterface.comp_end_state_service_name, callback_group=group_reentrant1)
        self._monitor_state = self.create_timer(1, self.monitor_state_callback,callback_group=group_mutex1)
        self.order_submit = ShipAndOrderSubmission(self, AriacInterface.submit_order_service_name, ship_cbg, robot_cbg)

        #Read and store order object instance
        self.read_store_orders=ReadStoreOrders(self,AriacInterface.order_topic1,self.order_queue,callback_group=group_reentrant1)
        self.sensor_read=SensorRead(self,callback_group=group_reentrant1)
        
        self.current_order_priority = False
        self.current_order = None
        self.pending_order = None

        # Transform listener to get the pose of some frames not predefined
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # client to start the competition
        self._start_competition_cli = self.create_client(
            Trigger, "/ariac/start_competition"
        )

        # client to move the floor robot to the home position
        self._move_robot_home_cli = self.create_client(
            Trigger, "/commander/move_robot_home", callback_group = group_reentrant1
        )

        # client to move a robot to a table
        self._move_robot_to_table_cli = self.create_client(
            MoveRobotToTable, "/commander/move_robot_to_table", callback_group = group_reentrant1
        )

        # client to move a robot to a table
        self._move_robot_to_tray_cli = self.create_client(
            MoveRobotToTray, "/commander/move_robot_to_tray", callback_group = group_reentrant1
        )

        # client to move a tray to an agv
        self._move_tray_to_agv_cli = self.create_client(
            MoveTrayToAGV, "/commander/move_tray_to_agv", callback_group = group_reentrant1
        )

        # client to move the end effector inside a tool changer
        self._enter_tool_changer_cli = self.create_client(
            EnterToolChanger, "/commander/enter_tool_changer", callback_group = group_reentrant1
        )

        # client to move the end effector outside a tool changer
        self._exit_tool_changer_cli = self.create_client(
            ExitToolChanger, "/commander/exit_tool_changer", callback_group = group_reentrant1
        )

        # client to activate/deactivate the vacuum gripper
        self._set_gripper_state_cli = self.create_client(
            VacuumGripperControl, "/ariac/floor_robot_enable_gripper", callback_group = group_reentrant1
        )

        # client to change the gripper type
        # the end effector must be inside the tool changer before calling this service
        self._change_gripper_cli = self.create_client(
            ChangeGripper, "/ariac/floor_robot_change_gripper", callback_group = group_reentrant1
        )

        # client to pick part
        # the end effector must be inside the tool changer before calling this service
        self._pick_part_cli = self.create_client(
            PickPart, "/commander/pick_part", callback_group = group_reentrant1
        )

        # client to place part
        # the end effector must be inside the tool changer before calling this service
        self._place_part_cli = self.create_client(
            PlacePart, "/commander/place_part", callback_group = group_reentrant1
        )

        self.create_subscription(
            VacuumGripperStateMsg,
            "/ariac/floor_robot_gripper_state",
            self.vacuum_gripper_state_cb,
            10,
            callback_group=robot_cbg,
        )
        self.vacuum_gripper_state = VacuumGripperStateMsg

        self.agv_tray_lock_cli = {}
        for num in range(1,5):
            self.agv_tray_lock_cli[num] = self.create_client(Trigger,f'/ariac/agv{num}_lock_tray',callback_group = group_reentrant1)



        # # The following flags are used to ensure an action is not triggered multiple times
        # self._moving_robot_home = False
        # self._moving_robot_to_table = False
        # self._entering_tool_changer = False
        # self._changing_gripper = False
        # self._exiting_tool_changer = False
        # self._activating_gripper = False
        self._deactivating_gripper = False
        # self._moving_robot_to_tray = False
        # self._moving_tray_to_agv = False
        # self._ending_demo = False

        # # The following flags are used to trigger the next action
        # self._kit_completed = False
        # self._competition_started = False
        # self._competition_state = None
        # self._moved_robot_home = False
        # self._moved_robot_to_table = False
        # self._entered_tool_changer = False
        # self._changed_gripper = False
        # self._exited_tool_changer = False
        # self._activated_gripper = False
        # self._deactivated_gripper = False
        # self._moved_robot_to_tray = False
        self._moved_tray_to_agv = False
        self._picked_part = False
        self._placed_part = False
        self._pick_part = False
        self._order_submitted = False


    def vacuum_gripper_state_cb(self, msg):
        self.vacuum_gripper_state = msg

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
            # self._order_submitted = False

            if not process_order.recievedOrder:
                # if not recieved the order details from sensor, then fetch it.
                try:
                    process_order.getOrder(order)
                except Exception as e:
                    self.get_logger().error(f"PROBLEM WITH THE GETING THE PARTS AND TRAY INFO FROM ENVIRONMENT!!!! \n {e}")
                    
            if not process_order.isOrderProcessed:
                # Start processing the order
                process_order.get_pick_place_position()
                    
            else: 
            
                try:
                    # Start shipping and submitting the order
                    self.get_logger().info(f"Starting the shipping and submitting the order {order.order_id}!!!")

                    data = self.order_submit.lock_move_agv(order)
                    if data is None:
                        self.get_logger().warn("Unable to ship!")

                    
                    self.current_order = self.pending_order
                    if self.pending_order is not None:
                        self.current_order_priority = self.pending_order[2]
                    self.pending_order = None
                except Exception as e:
                    self.get_logger().warn(f"Unable to ship and submit the order because of {e}!")
        else: 
            if self.comp_state.all_orders_recieved:
                time.sleep(25)
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

            self.current_order = (order, ProcessOrder(order=order,node=self), order.order_priority)
            self.get_logger().info(f"Got the High Priority Order!! Changing to it of id {self.current_order[0].order_id}!!")

        elif self.current_order is None and self.pending_order is None:
            self.current_order = (order, ProcessOrder(order=order,node=self), self.current_order_priority)
            self.get_logger().info(f"Got the Order!! Order Id : {order.order_id}")
        else:
            self.order_queue.appendleft(order)
