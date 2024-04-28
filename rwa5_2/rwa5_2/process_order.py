
from collections import deque
import numpy as np
import rclpy

from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException

FixQuadrantPositionsRelativeTray = {
    1 : np.array([-0.13, -0.08, 0]),
    2 : np.array([-0.13, 0.08, 0]),
    3 : np.array([0.13, -0.08, 0]),
    4 : np.array([0.13, 0.08, 0]),
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

        # get part info
        if len(parts_info) > 0:
            self._parts_info["parts_info"] = deque(parts_info)
            gripper_loc = self.get_grip_changer_pose("parts",parts_info[0]["kts"])
            if gripper_loc is None:
                raise Exception("Issue with the gripper change station location")
            self._parts_info["grip_changer_pose"] = gripper_loc
            agv_tray_loc = self.get_grip_changer_pose("parts",parts_info[0]["agv_num"])
            if agv_tray_loc is None:
                raise Exception("Issue with the agv tray location")
            self._parts_info["agv_tray_pose"] = agv_tray_loc

        # get tray info
        gripper_loc = self.get_grip_changer_pose("trays",tray_info["kts"])
        if gripper_loc is None:
            raise Exception("Issue with the gripper change station location")
        tray_info.update({"gripper_change_pose" : gripper_loc})

        agv_tray_loc = self.get_grip_changer_pose("trays",tray_info["agv_num"])
        if agv_tray_loc is None:
            raise Exception("Issue with the agv tray location")
        tray_info.update({"agv_tray_pose" : agv_tray_loc})

        self._tray_info = tray_info
        self._recievedOrder = True
        return True

    def get_pick_place_position(self):
        """
        Pick and Place position, part
        """
        if self._tray_info is not None:
            # pick and place tray call service here
            # pick_p = 1
            # if self._tray_info["pick"]:
            #     future = service.call(self._tray_info["pose"],pick)
            #      future.add_response_callback()   
            #      self._tray_info["pick"] = False
            # else:
            #     service.call(self._tray_info["agv_tray_pose"], place)
            #     self._tray_info = None
            pass
        elif self._parts_info is not None:
            # # pick and place part call service here
            # part_info = self._parts_info["parts_info"].popleft()
            # complete = False
            # if part_info["pick"]:
            #     service.call(self._tray_info["pose"],pick)
            #     part_info["pick"] = False
            # else:
            #     # place_pose compute relative agv_tray to quadrant
            #     # service.call()
            #     completed = True
            # self._parts_info["parts_info"].appendleft(part_info)
            pass
        
        return

    
    @property
    def isOrderProcessed(self) -> bool:
        """
        Return True if all the parts and tray are been put in place as given by order
        """
        if self._tray_info is None and self._parts_info is None:
            return True
        return False
    
    def pause(self):
        """
        Pause the process, service call to moveit, to pause
        """
        # Build the request
        request = Trigger.Request()
        response = self.pause_service.call(request)
        # Check the result of the service call.
        if response.success:
            self.node.get_logger().info(f'Successfully Paused the robot for order id {self._order_id}')
        else:
            self.node.get_logger().warn(response.message)
            raise Exception("Unable to Pause") 
            
        return True

    def get_grip_changer_pose(self, gripper_type, kts) -> Pose:
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
            # count+=1
        # if t is None:
        #     return None
        return  agv_tray_pose

