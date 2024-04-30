
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
    def __init__(self, order_id, node):
        """
        Contain the order information from sensor to pass it to the robot
        such as pick position and place position of the tray or part
        """
        self._order_id = order_id
        self.node = node
        
        # queue hold the part info, name, color, pick and place position
        self._parts_info = None # deque # list of dict containing all the important part info to robot know and pick and place it.
        self._tray_info = None # {} dictonary containing id, pose, agv_num, "pick" true or not details

        self._recievedOrder = False

        # place flag : monitor if the robot is placing the part or tray
        self._place = False 

        ## store all the order info in list
        self._order = deque()

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
    
    def getOrder(self, tray_info, parts_info):
        """
        Get Order information sequence wise from sensor data 
        first will be always be tray pick and place,
        then parts.
        
        Arguments:
            1. tray_info, name, pick position
            2. parts_info : [name,quadrant,pick position]
        """

        self._parts_info = {}
        self._tray_info = {}
        try:
            # get part info
            if len(parts_info) > 0:
                self._parts_info["parts_info"] = deque(parts_info)
                gripper_station_loc = self.get_gripper_station_pose("parts",parts_info[0]["kts"])
                if gripper_station_loc is None:
                    raise Exception("Issue with the gripper change station location")
                self._parts_info["gripper_station_pose"] = gripper_station_loc
                agv_tray_loc = self.get_agv_tray_pose(parts_info[0]["agv_num"])
                if agv_tray_loc is None:
                    raise Exception("Issue with the agv tray location")
                self._parts_info["agv_tray_pose"] = agv_tray_loc
                for part_info in parts_info:
                    self._order.appendleft(("parts",part_info))

            # get tray info
            gripper_station_loc = self.get_gripper_station_pose("trays",tray_info["kts"])
            if gripper_station_loc is None:
                raise Exception("Issue with the gripper change station location")
            tray_info.update({"gripper_station_pose" : gripper_station_loc})

            agv_tray_loc = self.get_agv_tray_pose(tray_info["agv_num"])
            if agv_tray_loc is None:
                raise Exception("Issue with the agv tray location")
            tray_info.update({"agv_tray_pose" : agv_tray_loc})

   
            self._order.appendleft(("tray",tray_info))

        except Exception as e:
            self.node.get_logger().error("ERROR : {}".format(traceback.format_exc()))

        self._recievedOrder = True
        return True

    # def _getPartOrder(self, part_info, types):
    #     request = 
    #     if types=="pick":
    #         request.tray_id = -1
    #         request.part_type =  TYPEOFPARTS[part_info["type"]]
    #         request.part_color = COLOROFPARTS[part_info["color"]]
    #         request.destination_pose = part_info["pose"]
    #         request.gripper_station_pose = self._parts_info["gripper_station_pose"]
    #         request.pick_place = PickPlace.Request().PICK

    #     else:

    #         request.tray_id = -1
    #         request.part_type = TYPEOFPARTS[part_info["type"]]
    #         request.part_color = COLOROFPARTS[part_info["color"]]

    #         quadrant = part_info["quadrant"]
    #         relative_pose = FixQuadrantPositionsRelativeTray[quadrant]

    #         request.destination_pose = Pose()
    #         request.destination_pose.position.x = self._parts_info["agv_tray_pose"].position.x + relative_pose[0]
    #         request.destination_pose.position.y = self._parts_info["agv_tray_pose"].position.y + relative_pose[1]
            
    #         request.gripper_station_pose = self._parts_info["gripper_station_pose"]
    #         request.pick_place = PickPlace.Request().PLACE

    #     return request
                
    # def _getTrayOrder(self, tray_info, types):
    #     request = PickPlace.Request()
    #     request.tray_id = tray_info["id"]
    #     request.part_type = ""
    #     if types=="pick":
    #         request.destination_pose = tray_info["pose"]
    #         request.gripper_station_pose = tray_info["gripper_station_pose"]
    #         request.pick_place = PickPlace.Request().PICK
            
    #     else:
    #         request.destination_pose = tray_info["agv_tray_pose"]
    #         request.gripper_station_pose = tray_info["gripper_station_pose"]
    #         request.pick_place = PickPlace.Request().PLACE

    #     return request


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
                    self.current_order = False
                    return 
                #     # if self._moved_robot_home:
                #     # if not self._moving_robot_to_table:
                #     RM._move_robot_to_table(self.node,order["kts"])
                
                #     self.node.get_logger().info(f"current gripper type {self.node.vacuum_gripper_state.type}") 
                #     if types_of_gripper[self.node.vacuum_gripper_state.type] != ChangeGripper.Request.TRAY_GRIPPER:
                #         self.node.get_logger().info("Moving to gripper change station")                        
                #         RM._enter_tool_changer(self.node, f"kts{order['kts']}", "trays")

                #         # if self.node._entered_tool_changer:
                #         #     if not self.node._changing_gripper:
                #         RM._change_gripper(self.node,ChangeGripper.Request.TRAY_GRIPPER)

                #         RM._exit_tool_changer(self.node,f"kts{order['kts']}", "trays")
                            
                #     if not self.node.vacuum_gripper_state.enabled:
                #         RM._activate_gripper(self.node)
                # # move to tray
                #     RM._move_robot_to_tray(self.node,order["id"], order["pose"])
                #     # if self.node.vacuum_gripper_state.attached:
                #     RM._move_tray_to_agv(self.node,order["agv_num"])
                #     if self.node._moved_tray_to_agv:
                #         RM._deactivate_gripper(self.node)
                #     self.current_order = False
                    # pass
                else:
                    self.node.get_logger().info("Picking Order") 
                    
                    if types_of_gripper[self.node.vacuum_gripper_state.type] != ChangeGripper.Request.PART_GRIPPER:
                        self.node.get_logger().info("Moving to gripper change station")                        
                    
                        RM._move_robot_to_table(self.node,order["kts"])
                        RM._enter_tool_changer(self.node, f"kts{order['kts']}", "parts")

                        # if self.node._entered_tool_changer:
                        #     if not self.node._changing_gripper:
                        RM._change_gripper(self.node,ChangeGripper.Request.PART_GRIPPER)

                        RM._exit_tool_changer(self.node,f"kts{order['kts']}", "parts")
                    if not self.node.vacuum_gripper_state.enabled:
                        RM._activate_gripper(self.node)
                    RM._pick_part(self.node, order["type"], order["color"], order["pose"])
                    if self.node._picked_part:
                        RM._place_part(self.node, order["agv_num"], order["quadrant"])
                    if self.node._placed_part:
                        RM._deactivate_gripper(self.node)
                    self.current_order = False
                # if order_pick is None:
                #     current_order = order_place #self._order.popleft()[1]
                #     # self.finish_1_order = False
                #     self.order_type="place"
                # else:
                #     current_order = order_pick
                #     self._order.appendleft((None,order_place))
                # self.node.get_logger().info(f"Request {current_order}")
                # future = self.node.move_service.call_async(current_order)
                
                # future.add_done_callback(partial(self.add_response_callback,info=current_order))  # Add response callback
                        
            else:
                # do nothing as wait for previous order to process
                pass

        except Exception as e:
            self.node.get_logger().error("ERROR : {}".format(traceback.format_exc()))

        return  
    
    def add_response_callback(self, future, info):
        """
        response callback function for handling success and message.
        """
        # Check if the future has a result
        if future.done() and not future.cancelled():
            result = future.result()
            # Check if the result is successful
            if result.success:
                self.node.get_logger().info(f"Service call successful:{result.message}")
                # Handle success scenario here

            else:
                self.node.get_logger().error(f"Service call failed: {result.message}")

                ## Regain the info for use when the order process resumes
                if info.pick_place==PickPlace.Request().PICK:
                    pick_o,place_o = self._order.popleft()
                    self._order.appendleft((pick_o,place_o))

                else:
                    self._order.appendleft((None,info))

            
                # Handle failure scenario here
        else:
            self.node.get_logger().error(f"Future was cancelled or did not complete successfully")
            ## Regain the info for use when the order process resumes
            if info.pick_place==PickPlace.Request().PICK:
                pick_o,place_o = self._order.popleft()
                self._order.appendleft((pick_o,place_o))

            else:
                self._order.appendleft((None,info))

        self._place = False
        self.current_order= False

    @property
    def isOrderProcessed(self) -> bool:
        """
        Return True if all the parts and tray are been put in place as given by order
        """

        return len(self._order)==0
    
    def pause(self):
        """
        Pause the process, service call to moveit, to pause
        """
        try:
            if self.current_order == False:
                return True
            # self.node.get_logger().info("Pausing the order.")
            # if self.order_type=="pick":
            #     self.node.get_logger().info("Currently picking the order")
            #     while self.current_order:
            #         continue
            #     self.node.get_logger().info("Start placing the Order.")
            #     self.get_pick_place_position()

            # while self.current_order and self.order_type=="place":
            #     continue
            # self.node.get_logger().info("Completed 1 order. Now pausing!!")
        except Exception as e:
            self.node.get_logger().error("ERROR : {}".format(traceback.format_exc()))
            return False

        # # Build the request
        # request = Trigger.Request()
        # response = self.pause_service.call_async(request)
        # # Check the result of the service call.
        # if response.success:
        #     self.node.get_logger().info(f'Successfully Paused the robot for order id {self._order_id}')
        # else:
        #     self.node.get_logger().warn(response.message)
        #     raise Exception("Unable to Pause") 
            
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
