
from collections import deque
import numpy as np
import rclpy

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

FixAGVPosition = {
    1 : np.array([0.4,0,0]),
    2 : np.array([0.4,0,0]),
    3 : np.array([0.4,0,0]),
    4 : np.array([0.4,0,0])
}

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


        pass
    
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
            loc = self.get_grip_changer_pose("parts",parts_info[0]["kts"])
            if loc is None:
                raise Exception("Issue with the gripper change station location")
            self._parts_info["grip_changer_pose"] = loc

        # get tray info
        loc = self.get_grip_changer_pose("trays",tray_info["kts"])
        if loc is None:
            raise Exception("Issue with the gripper change station location")
        self._tray_info = tray_info.update({"grip_changer_pose" : loc})
        self.node.get_logger().info(f"Tray Info : {self._tray_info}")
        self._recievedOrder = True
        return True

    def get_pick_place_position(self):
        """
        Pick and Place position, part
        """
        if self._tray_info is not None:
            # pick and place tray call service here
            pass
        elif self._parts_info is not None:
            # pick and place part call service here
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
        from_frame_rel = f"kts{kts}_tool_changer_{gripper_type}_frame"
        gripper_changer_pose = Pose()
        try:
            t = self.node.tf_buffer.lookup_transform(
                to_frame_rel,  # referred to world coordinate
                from_frame_rel, # example changing the gripper tool from kts1 station
                rclpy.time.Time())
            (gripper_changer_pose.position.x, gripper_changer_pose.position.y, gripper_changer_pose.position.z) = (t.transform.translation.x,
                                                                                                                        t.transform.translation.y,
                                                                                                                            t.transform.translation.z) 
            gripper_changer_pose.orientation.x = t.transform.translation.x
            gripper_changer_pose.orientation.y = t.transform.translation.y
            gripper_changer_pose.orientation.z = t.transform.rotation.z
            gripper_changer_pose.orientation.w = t.transform.rotation.w

        except TransformException as ex:
            self.node.get_logger().error(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                # return None
            # count+=1
        if t is None:
            return None
        return gripper_changer_pose


