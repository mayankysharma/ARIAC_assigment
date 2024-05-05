
from collections import deque
import numpy as np
import rclpy
from functools import partial
import traceback
import sys

from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException

from ariac_msgs.srv import ChangeGripper
# Import custom ROS services

import robot_move as RM
from utils import COLOROFPARTS, TYPEOFPARTS

FixQuadrantPositionsRelativeTray = {
    1 : np.array([-0.13, -0.08, 0]),
    2 : np.array([-0.13, 0.08, 0]),
    3 : np.array([0.13, -0.08, 0]),
    4 : np.array([0.13, 0.08, 0]),
}


types_of_gripper = {
    "part_gripper" : ChangeGripper.Request.PART_GRIPPER,
    "tray_gripper" : ChangeGripper.Request.TRAY_GRIPPER
}

# FixAGVPosition = {
#     1 : np.array([0.4,0,0]),
#     2 : np.array([0.4,0,0]),
#     3 : np.array([0.4,0,0]),
#     4 : np.array([0.4,0,0])
# }

class ProcessOrder():
    def __init__(self, order, node):
        """
        Contain the order information from sensor to pass it to the robot
        such as pick position and place position of the tray or part

        Args:
            order_id (int): The ID of the order.
            node: The node object used for communication and logging.
        """
        self._order_id = order.order_id
        self.node = node
        
        # queue hold the part info, name, color, pick and place position
        self._parts_info = None # deque # list of dict containing all the important part info to robot know and pick and place it.
        self._tray_info = None # {} dictonary containing id, pose, agv_num, "pick" true or not details

        self._recievedOrder = False

        # place flag : monitor if the robot is placing the part or tray
        self._place = False 

        ## store all the order info in list
        self.getOrder(order)

        ## check if any order part in process
        self.current_order= False
    
        # Check if the order is finish
        self.order_type = "pick"

    @property
    def recievedOrder(self):
        """
        This is just an attribute to check if have recieved all the information of the order.
        """
        return self._recievedOrder
    
    def getOrder(self, order):
        """
            Get order data, and return a list of information of part and tray i
        Args:
            order : class Order (utils.py)
        Returns:
            deque()
        example:

        """
        self._order = deque()
        try:
            self._order.append(("tray", {
                "tray_id" : order.order_task.tray_id,
                "agv_num" : order.order_task.agv_number,
            }))

            for part in order.order_task.parts:
                self._order.append((
                    "part", {
                        "quadrant" : part.quadrant,
                        "type" : part.part.type,
                        "color" : part.part.color,
                        "agv_num" : order.order_task.agv_number,   
                    }
                ))

        except Exception as e:
            self.node.get_logger().error("ERROR : {}".format(traceback.format_exc()))
            # return deque()
            self._recievedOrder = False
            return
        self._recievedOrder = True
        # return _order

    def get_pick_place_position(self):
        """
        Pick and Place position, part
        """
        try:
            if not self.current_order:
                self.node.get_logger().info(f"Processing the order {self._order_id}!!")
                self.current_order = True
                types, order = self._order.popleft()
                self.node.get_logger().info(f"Processing {types}") 
                if types=="tray":
                    tray_info = self.node.sensor_read.get_tray_pose_from_sensor(order["tray_id"],verbose=True)

                    if not RM._move_robot_to_table(self.node,tray_info["kts"]):
                        self.current_order = False
                        self._order.appendleft((types,order))
                        return                

                    self.node.get_logger().info(f"current gripper type {self.node.vacuum_gripper_state.type}") 
                    if types_of_gripper[self.node.vacuum_gripper_state.type] != ChangeGripper.Request.TRAY_GRIPPER:
                        self.node.get_logger().info("Moving to gripper change station")                        
                        if not RM._enter_tool_changer(self.node, f"kts{tray_info['kts']}", "trays"):
                            self.current_order = False
                            self._order.appendleft((types,order))
                            return

                        if not RM._change_gripper(self.node,ChangeGripper.Request.TRAY_GRIPPER):
                            self.current_order = False
                            self._order.appendleft((types,order))
                            return
                            
                        if not RM._exit_tool_changer(self.node,f"kts{tray_info['kts']}", "trays"):
                            self.current_order = False
                            self._order.appendleft((types,order))
                            return
                        
                    if not self.node.vacuum_gripper_state.enabled:
                        if not RM._activate_gripper(self.node):
                            self.current_order = False
                            self._order.appendleft((types,order))
                            return

                    if not RM._move_robot_to_tray(self.node,tray_info["tray_id"], tray_info["pose"]):
                        self.current_order = False
                        self._order.appendleft((types,order))
                        return

                    if not RM._move_tray_to_agv(self.node,order["agv_num"]):
                        self.current_order = False
                        self._order.appendleft((types,order))
                        return
                    
                    if self.node._moved_tray_to_agv:
                        if not RM._deactivate_gripper(self.node):
                            self.current_order = False
                            self._order.appendleft((types,order))
                            return

                    if not RM.agv_tray_locked(self.node,order["agv_num"]):
                        self.current_order = False
                        self._order.appendleft((types,order))
                        return

                    self.current_order = False

                else:
                    self.node.get_logger().info("Picking Order") 
                    part_info = self.node.sensor_read.get_part_pose_from_sensor(part_color=order["color"], part_type=order["type"])

                    if types_of_gripper[self.node.vacuum_gripper_state.type] != ChangeGripper.Request.PART_GRIPPER:
                        self.node.get_logger().info("Moving to gripper change station")                        
                    
                        if not RM._move_robot_to_table(self.node,part_info["kts"]):
                            self.current_order = False
                            self._order.appendleft((types,order))
                            return

                        if not RM._enter_tool_changer(self.node, f"kts{part_info['kts']}", "parts"):
                            self.current_order = False
                            self._order.appendleft((types,order))
                            return


                        if not RM._change_gripper(self.node,ChangeGripper.Request.PART_GRIPPER):
                            self.current_order = False
                            self._order.appendleft((types,order))
                            return

                        if not RM._exit_tool_changer(self.node,f"kts{part_info['kts']}", "parts"):
                            self.current_order = False
                            self._order.appendleft((types,order))
                            return
                    
                    if not self.node.vacuum_gripper_state.enabled:
                        if not RM._activate_gripper(self.node):
                            self.current_order = False
                            self._order.appendleft((types,order))
                            return
                    
                    if not RM._pick_part(self.node, order["type"], order["color"], part_info["pose"]):
                        self.current_order = False
                        self._order.appendleft((types,order))
                        return

                    if self.node._picked_part:
                        # if not self.node.vaccum_gripper_state.attached:
                        #     self.current_order = False
                        #     self._order.appendleft((types,order))
                        if not RM._place_part(self.node, order["agv_num"], order["quadrant"]):
                                self.current_order = False
                                self._order.appendleft((types,order))
                                return

                        # if self.node.vacuum_gripper_state.enabled:
                        #     if not RM._deactivate_gripper(self.node):
                        #         self.current_order = False
                        #         self._order.appendleft((types,order))
                        #         return
                    
                    self.current_order = False
                    #Faulty gripper condition to be implemented
                    # Check the floor_robot_gripper_state
                    
                    # If attached false:
                        # set current order false
                        # Check the AGV cameras if the part is dropped on them, if yes pick it from AGV
                        # If not, check bin camera for the same part
                        # If available pick from bin and put the part back on the specified order
            else:
                # do nothing as wait for previous order to process
                pass

        except Exception as e:
            self.node.get_logger().error("ERROR : {}".format(traceback.format_exc()))

        return  
    
    @property
    def isOrderProcessed(self) -> bool:
        """
        Return True if all the parts and tray are been put in place as given by order
        """
        self.node.get_logger().info(f"Order Processed : {len(self._order)==0}")

        return len(self._order)==0
    
    def pause(self):
        """
        Pause the process, service call to moveit, to pause
        """
        try:
            while self.current_order: continue
            return True

        except Exception as e:
            ...
            self.node.get_logger().error("ERROR : {}".format(traceback.format_exc()))
            return False


        return True

    def get_gripper_station_pose(self, gripper_type, kts) -> Pose:
        """
        This Function get the pose of the gripper change station, so that robot can move there and change
        there according to task (defined as gripper type)
        Args:
            gripper_type : trays or parts
            kts: 1 or 2
        Return:
            pose: of the location.
        """
        t = None
        count = 0
        # while t is None and count<10:
        to_frame_rel = "world"
        # from_frame_rel = "kts1_tool_changer_parts_frame" 
        from_frame_rel =  f"kts{kts}_tool_changer_{gripper_type}_frame"
        gripper_changer_pose = Pose()
        # self.node.get_logger().info(f"pose : {gripper_changer_pose}")
        try:
            t = self.node.tf_buffer.lookup_transform(
                to_frame_rel,  # referred to world coordinate
                from_frame_rel, # example changing the gripper tool from kts1 station
                rclpy.time.Time())
            # self.node.get_logger().info(f"transform : {t}")
            (gripper_changer_pose.position.x, gripper_changer_pose.position.y, gripper_changer_pose.position.z) = (t.transform.translation.x,
                                                                                                                        t.transform.translation.y,
                                                                                                                            t.transform.translation.z) 
            gripper_changer_pose.orientation.x = t.transform.translation.x
            gripper_changer_pose.orientation.y = t.transform.translation.y
            gripper_changer_pose.orientation.z = t.transform.rotation.z
            gripper_changer_pose.orientation.w = t.transform.rotation.w
            # self.node.get_logger().info(f"gripper pose : {gripper_changer_pose}")

        except TransformException as ex:
            self.node.get_logger().error(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return None
            # count+=1
        # if t is None:
        #     return None
        return gripper_changer_pose

    def get_agv_tray_pose(self, agv_num) -> Pose:
        """
        This Function get the pose of the agv tray pose to place the tray on agv.
        Args:
            agv_num: b/w [1,2,3,4]
        Return:
            pose: agv tray pose.
        """
        t = None
        count = 0
        # while t is None and count<10:
        to_frame_rel = "world"
        from_frame_rel =  f"agv{agv_num}_tray"

        agv_tray_pose = Pose()
        # self.node.get_logger().info(f"pose :  agv_tray_pose}")
        try:
            t = self.node.tf_buffer.lookup_transform(
                to_frame_rel,  # referred to world coordinate
                from_frame_rel, # example changing the gripper tool from kts1 station
                rclpy.time.Time())
            # self.node.get_logger().info(f"transform : {t}")
            (agv_tray_pose.position.x,   agv_tray_pose.position.y,  agv_tray_pose.position.z) = (t.transform.translation.x,
                                                                                                                        t.transform.translation.y,
                                                                                                                            t.transform.translation.z) 
            agv_tray_pose.orientation.x = t.transform.translation.x
            agv_tray_pose.orientation.y = t.transform.translation.y
            agv_tray_pose.orientation.z = t.transform.rotation.z
            agv_tray_pose.orientation.w = t.transform.rotation.w
            # self.node.get_logger().info(f"gripper pose :  agv_tray_pose}")

        except TransformException as ex:
            self.node.get_logger().error(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return None

        return  agv_tray_pose
