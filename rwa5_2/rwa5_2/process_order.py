
from collections import deque
import numpy as np

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
        if len(parts_info) > 0:
            self._parts_info = deque(parts_info)
        self._tray_info = tray_info
        
        self._recievedOrder = True
        

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

