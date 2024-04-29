import math
from typing import List, Tuple
from dataclasses import dataclass
import PyKDL
from geometry_msgs.msg import (
    Pose,
    PoseStamped, 
    Vector3,
    Quaternion
)
from ariac_msgs.msg import (
    PartPose as PartPoseMsg,
    KitTrayPose as KitTrayPoseMsg,
    Part as PartMsg,
    Order as OrderMsg
)

def Mult_pose(Pose_1: Pose, Pose_2: Pose) -> Pose:
    '''
    Using KDL to multiply two poses together.

    This function takes two Pose objects and calculates the result of multiplying them together using KDL (Kinematics and Dynamics Library).

    Args:
        Pose_1 (Pose): Pose of the first frame
        Pose_2 (Pose): Pose of the second frame

    Returns:
        Final_Pose (Pose): Pose of the resulting frame
    '''

    # Extract orientation and position components from Pose_1
    Orientation_Frame_1 = Pose_1.orientation
    Frame1 = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(Orientation_Frame_1.x, Orientation_Frame_1.y, Orientation_Frame_1.z, Orientation_Frame_1.w),
        PyKDL.Vector(Pose_1.position.x, Pose_1.position.y, Pose_1.position.z))

    # Extract orientation and position components from Pose_2
    Orientation_Frame_2 = Pose_2.orientation
    Frame2 = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(Orientation_Frame_2.x, Orientation_Frame_2.y, Orientation_Frame_2.z, Orientation_Frame_2.w),
        PyKDL.Vector(Pose_2.position.x, Pose_2.position.y, Pose_2.position.z))

    # Multiply the two frames together to get the resulting frame
    Resulting_Frame3 = Frame1 * Frame2

    # Extract position components from the resulting frame and assign to Final_pose
    Final_pose = Pose()
    Final_pose.position.x = Resulting_Frame3.p.x()
    Final_pose.position.y = Resulting_Frame3.p.y()
    Final_pose.position.z = Resulting_Frame3.p.z()

    # Extract quaternion components from the resulting frame and assign to Final_pose
    Quaternion_orientation = Resulting_Frame3.M.GetQuaternion()
    Final_pose.orientation.x = Quaternion_orientation[0]
    Final_pose.orientation.y = Quaternion_orientation[1]
    Final_pose.orientation.z = Quaternion_orientation[2]
    Final_pose.orientation.w = Quaternion_orientation[3]

    # Return the final pose
    return Final_pose

def Quart_to_RPY(quart: Quaternion) -> Tuple[float, float, float]:
    ''' 
    Use KDL to convert a quaternion to euler angles roll, pitch, yaw.
    Args:
        q (Quaternion): quaternion to convert
    Returns:
        Tuple[float, float, float]: roll, pitch, yaw
    '''
    
    RPY = PyKDL.Rotation.Quaternion(quart.x, quart.y, quart.z, quart.w)
    return RPY.GetRPY()

def RPY_to_Quart(rpy) -> Tuple[float, float, float, float]:
    ''' 
    Use KDL to convert a quaternion to euler angles roll, pitch, yaw.
    Args:
        q (Quaternion): quaternion to convert
    Returns:
        Tuple[float, float, float]: roll, pitch, yaw
    '''
    
    RPY = PyKDL.Rotation.RPY(rpy[0],rpy[1],rpy[2])  #Quaternion(quart.x, quart.y, quart.z, quart.w)
    return RPY.GetQuaternion()

def RAD_TO_DEGREE(radians: float) -> str:
    '''
    Converts radians to degrees in the domain [-PI, PI]
    Args:
        radians (float): value in radians
    Returns:
        str: String representing the value in degrees
    '''
    
    degrees = math.degrees(radians)
    if degrees > 180:
        degrees = degrees - 360
    elif degrees < -180:
        degrees = degrees + 360

    if -1 < degrees < 1:
        degrees = 0 
    
    return f'{degrees:.0f}' + chr(176)

@dataclass
class AdvancedLogicalCameraImage:
    '''
    Class to store information about a AdvancedLogicalCameraImageMsg
    '''
    _part_poses: PartPoseMsg
    _tray_poses: KitTrayPoseMsg
    _sensor_pose: Pose
@dataclass
class KittingPart:
    ''' 
    Class to store information about a kitting part message.
    '''

    _quadrant: int
    _part: PartMsg

    @property
    def quadrant(self) -> int:
        ''' 
        Returns the quadrant of the part.

        Returns:
            int: The quadrant of the part.
        '''
        return self._quadrant

    @property
    def part(self) -> PartMsg:
        ''' 
        Returns the type of the part.

        Returns:
            PartMsg: The type of the part.
        '''
        return self._part

@dataclass
class KittingTask:
    ''' 
    Class to store information about a kitting task message.
    '''

    _agv_number: int
    _tray_id: int
    _destination: int
    _parts:  List[KittingPart]

    @property
    def agv_number(self) -> int:
        ''' 
        Returns the AGV number.

        Returns:
            int: The AGV number.
        '''
        return self._agv_number

    @property
    def tray_id(self) -> int:
        ''' 
        Returns the tray ID.

        Returns:
            int: The tray ID.
        '''
        return self._tray_id

    @property
    def destination(self) -> int:
        ''' 
        Returns the destination.

        Returns:
            int: The destination.
        '''
        return self._destination

    @property
    def parts(self) -> List[KittingPart]:
        ''' 
        Returns the list of parts.

        Returns:
            List[KittingPart]: The list of parts.
        '''
        return self._parts

class Order:
    ''' 
    Class to store one order message from the topic /ariac/orders.
    '''

    def __init__(self, msg: OrderMsg) -> None:
        ''' 
        Initialize the Order object.

        Args:
            msg (OrderMsg): The order message.
        '''
        self.order_id = msg.id
        self.order_type = msg.type
        self.order_priority = msg.priority

        if self.order_type == OrderMsg.KITTING:
            self.order_task = KittingTask(msg.kitting_task.agv_number,
                                          msg.kitting_task.tray_id,
                                          msg.kitting_task.destination,
                                          msg.kitting_task.parts)
        else:
            self.order_task = None
