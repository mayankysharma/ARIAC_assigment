from std_srvs.srv import Trigger
from ariac_msgs.srv import ChangeGripper, VacuumGripperControl

from functools import partial

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

def _move_robot_home(self, end_demo=False):
    """
    Move the floor robot to its home position
    """

    self.get_logger().info("👉 Moving robot home...")
    if end_demo:
        self._ending_demo = True
    else:
        self._moving_robot_home = True

    while not self._move_robot_home_cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info("Service not available, waiting...")

    request = Trigger.Request()
    future = self._move_robot_home_cli.call(request)
    return _move_robot_home_done_cb(self.node, future)

def _move_robot_home_done_cb(self, future):
    """
    Client callback for the service /competitor/floor_robot/go_home

    Args:
        future (Future): A future object
    """
    message = future.message
    if future.success:
        self.get_logger().info(f"✅ {message}")
        return True
    else:
        self.get_logger().fatal(f"💀 {message}")
        return False

def _move_robot_to_table(self, table_id):
    """
    Move the floor robot to a table

    Args:
        table_id (int): 1 for kts1 and 2 for kts2
    """

    self.get_logger().info("👉 Moving robot to changing station...")
    self._moving_robot_to_table = True
    while not self._move_robot_to_table_cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info("Service not available, waiting...")

    request = MoveRobotToTable.Request()
    request.kts = table_id
    future = self._move_robot_to_table_cli.call(request)
    return _move_robot_to_table_done_cb(self, future)

def _move_robot_to_table_done_cb(self, future):
    """
    Client callback for the service /commander/move_robot_to_table

    Args:
        future (Future): A future object
    """
    message = future.message
    if future.success:
        self.get_logger().info(f"✅ {message}")
        self._moved_robot_to_table = True
        return True
    else:
        self.get_logger().fatal(f"💀 {message}")
        return False

def _enter_tool_changer(self, station, gripper_type):
    """
    Move the end effector inside a tool changer

    Args:
        station (str): 'kts1' or 'kts2'
        gripper_type (str): 'parts' or 'trays'
    """
    self.get_logger().info("👉 Entering tool changer...")
    self._entering_tool_changer = True
    while not self._enter_tool_changer_cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info("Service not available, waiting...")

    request = EnterToolChanger.Request()
    request.changing_station = station
    request.gripper_type = gripper_type
    future = self._enter_tool_changer_cli.call(request)
    return _enter_tool_changer_done_cb(self,future)

def _enter_tool_changer_done_cb(self, future):
    """
    Client callback for the service /commander/enter_tool_changer

    Args:
        future (Future): A future object
    """
    message = future.message
    if future.success:
        self.get_logger().info(f"✅ {message}")
        self._entered_tool_changer = True
        return  True
    else:
        self.get_logger().fatal(f"💀 {message}")
        return False

def _change_gripper(self, gripper_type):
    """
    Change the gripper

    Args:
        station (str): 'kts1' or 'kts2'
        gripper_type (str): 'parts' or 'trays'
    """
    self.get_logger().info("👉 Changing gripper...")
    self._changing_gripper = True
    while not self._change_gripper_cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info("Service not available, waiting...")

    request = ChangeGripper.Request()
    request.gripper_type = gripper_type
    future = self._change_gripper_cli.call(request)
    return _change_gripper_done_cb(self,future)

def _change_gripper_done_cb(self, future):
    """
    Client callback for the service /ariac/floor_robot_change_gripper

    Args:
        future (Future): A future object
    """
    message = future.message
    if future.success:
        self.get_logger().info("✅ Gripper changed")
        self._changed_gripper = True
        return  True
    else:
        self.get_logger().fatal(f"💀 {message}")
        return  False

def _exit_tool_changer(self, station, gripper_type):
    """
    Move the end effector outside a tool changer

    Args:
        station (str): 'kts1' or 'kts2'
        gripper_type (str): 'parts' or 'trays'
    """
    self.get_logger().info("👉 Exiting tool changer...")
    self._exiting_tool_changer = True
    while not self._exit_tool_changer_cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info("Service not available, waiting...")

    request = ExitToolChanger.Request()
    request.changing_station = station
    request.gripper_type = gripper_type
    future = self._exit_tool_changer_cli.call(request)
    return _exit_tool_changer_done_cb(self,future)

def _exit_tool_changer_done_cb(self, future):
    """
    Client callback for the service /commander/exit_tool_changer

    Args:
        future (Future): A future object
    """
    message = future.message
    if future.success:
        self.get_logger().info(f"✅ {message}")
        self._exited_tool_changer = True
        return True
    else:
        self.get_logger().fatal(f"💀 {message}")
        return False

def _activate_gripper(self):
    """
    Activate the gripper
    """
    self.get_logger().info("👉 Activating gripper...")
    self._activating_gripper = True
    
    while not self._set_gripper_state_cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info("Service not available, waiting...")

    request = VacuumGripperControl.Request()
    request.enable = True
    future = self._set_gripper_state_cli.call(request)
    return _activate_gripper_done_cb(self,future)

def _activate_gripper_done_cb(self, future):
    """
    Client callback for the service /ariac/floor_robot_enable_gripper

    Args:
        future (Future): A future object
    """
    if future.success:
        self.get_logger().info("✅ Gripper activated")
        self._activated_gripper = True  
        return  True
    else:
        self.get_logger().fatal("💀 Gripper not activated")
        return False

def _deactivate_gripper(self):
    """
    Deactivate the gripper
    """
    self.get_logger().info("👉 Deactivating gripper...")
    self._deactivating_gripper = True
    while not self._set_gripper_state_cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info("Service not available, waiting...")

    request = VacuumGripperControl.Request()
    request.enable = False
    future = self._set_gripper_state_cli.call(request)
    return _deactivate_gripper_done_cb(self,future)

def _deactivate_gripper_done_cb(self, future):
    """
    Client callback for the service /ariac/floor_robot_enable_gripper

    Args:
        future (Future): A future object
    """
    if future.success:
        self.get_logger().info("✅ Gripper deactivated")
        # self._deactivated_gripper = True
        self._picked_part = False
        self._deactivating_gripper = False
        return True
    else:
        self.get_logger().fatal("💀 Gripper not deactivated")
        return False

def _move_robot_to_tray(self, tray_id, tray_pose):
    """
    Move the floor robot to a tray to pick it up
    """
    self.get_logger().info("👉 Moving robot to tray...")
    self._moving_robot_to_tray = True
    
    while not self._move_robot_to_tray_cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().error("Service not available, waiting...")

    request = MoveRobotToTray.Request()
    request.tray_id = tray_id
    request.tray_pose_in_world = tray_pose
    future = self._move_robot_to_tray_cli.call(request)
    return _move_robot_to_tray_done_cb(self,future)

def _move_robot_to_tray_done_cb(self, future):
    """
    Client callback for the service /commander/move_robot_to_tray

    Args:
        future (Future): A future object
    """
    message = future.message
    if future.success:
        self.get_logger().info(f"✅ {message}")
        self._moved_robot_to_tray = True
        return True
    else:
        self.get_logger().fatal(f"💀 {message}")
        return False

# @brief Move the floor robot to its home position
def _move_tray_to_agv(self, agv_number):
    
    self.get_logger().info("👉 Moving tray to AGV...")
    self._moving_tray_to_agv = True

    while not self._move_tray_to_agv_cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info("Service not available, waiting...")

    request = MoveTrayToAGV.Request()
    request.agv_number = agv_number
    future = self._move_tray_to_agv_cli.call(request)
    return _move_tray_to_agv_done_cb(self,future)

def _move_tray_to_agv_done_cb(self, future):
    """
    Client callback for the service /commander/move_tray_to_agv

    Args:
        future (Future): A future object
    """
    message = future.message
    if future.success:
        self.get_logger().info(f"✅ {message}")
        self._moved_tray_to_agv = True
        return  True
    else:
        self.get_logger().fatal(f"💀 {message}")
        return  False

# @brief Move the floor robot to its home position
def _pick_part(self, part_type, part_color, part_pose, bin_side, agv_num):
    
    self.get_logger().info("👉 Picking Part...")
    # self._moving_tray_to_agv = True

    while not self._pick_part_cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info("Service not available, waiting...")

    request = PickPart.Request()
    request.part_type = part_type
    request.part_color = part_color
    request.part_pose_in_world = part_pose
    request.bin_side = bin_side
    request.agv_num = agv_num
    future = self._pick_part_cli.call(request)
    return _pick_part_done_cb(self,future)

def _pick_part_done_cb(self, future):
    """
    Client callback for the service /commander/move_tray_to_agv

    Args:
        future (Future): A future object
    """
    message = future.message
    if future.success:
        self.get_logger().info(f"✅ {message}")
        self._picked_part = True
        return True
    else:
        self._picked_part = False
        self.get_logger().fatal(f"💀 {message}")
        return False
# @brief Move the floor robot to its home position
def _place_part(self, agv_num, quadrant):
    
    self.get_logger().info("👉 Placing Part...")
    # self._moving_tray_to_agv = True

    while not self._place_part_cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info("Service not available, waiting...")

    request = PlacePart.Request()
    request.agv_num = agv_num
    request.quadrant = quadrant
    future = self._place_part_cli.call(request)
    return _pick_part_done_cb(self,future)

def _place_part_done_cb(self, future):
    """
    Client callback for the service /commander/move_tray_to_agv

    Args:
        future (Future): A future object
    """
    message = future.message
    self._placed_part = True 
    if future.success:
        self.get_logger().info(f"✅ {message}")
        self._deactivating_gripper = True
        return True
    else:
        self.get_logger().fatal(f"💀 {message}")
        return False

def agv_tray_locked(self, num):
        '''
        Lock the tray of an AGV and parts on the tray. This will prevent tray and parts from moving during transport.

        Args:
            num (int):  AGV number

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''

        # Build the request
        request = Trigger.Request()
        # Send the request
        future = self.agv_tray_lock_cli[num].call(request)

        # Check the response
        if future.success:
            self.get_logger().info(f'AGV{num}\'s tray locked')
            return True
        else:
            self.get_logger().warn('Unable to lock tray')
            raise Exception("Unable to lock the tray")
        return False