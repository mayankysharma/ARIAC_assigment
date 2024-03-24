import math
from typing import List, Tuple
from dataclasses import dataclass

from ariac_msgs.msg import (
    PartPose as PartPoseMsg,
    KitTrayPose as KitTrayPoseMsg,
    Part as PartMsg,
    Order as OrderMsg
)



@dataclass
class KittingPart:
    '''
    Class to store information about a KittingPartMsg.
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
    Class to store information about a KittingTaskMsg.
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