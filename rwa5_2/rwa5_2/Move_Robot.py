#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from custom_msgs.srv import (
    MoveRobotToTable,
    MoveRobotToTray,
    MoveTrayToAGV,
)


from geometry_msgs.msg import Pose
class Move_Robot():
    def __init__(self,node):
        
        sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])
        main_timer_cb_group = MutuallyExclusiveCallbackGroup()
        # The subscriber callback group is used to make the subscriber callback mutually exclusive
        # All subscriber callbacks are called in the same thread
        subscriber_cb_group = MutuallyExclusiveCallbackGroup()    
        
    
        #create clients to
        self._move_floor_robot_home = self.create_client(
            Trigger, '/competitor/move_floor_robot_home')
        self._move_robot_to_table_cli = self.create_client(
            MoveRobotToTable, "/commander/move_robot_to_table"
        )
        # client to move a robot to a table
        self._move_robot_to_tray_cli = self.create_client(
            MoveRobotToTray, "/commander/move_robot_to_tray"
        )

        # client to move a tray to an agv
        self._move_tray_to_agv_cli = self.create_client(
            MoveTrayToAGV, "/commander/move_tray_to_agv"
        )
        self._moving_robot_home = False

        self._main_timer = self.create_timer(
            1, self._main_timer_cb, callback_group=main_timer_cb_group
        )
        
        # The following flags are used to ensure an action is not triggered multiple times
        self._moving_robot_home = False
        self._moving_robot_to_table = False
        self._moving_robot_to_tray = False
        self._moving_tray_to_agv = False
        self._ending_demo = False

    def move_robot_home(self, robot_name):
        '''Move one of the robots to its home position.

        Arguments:
            robot_name -- Name of the robot to move home
        '''
        request = Trigger.Request()

        if robot_name == 'floor_robot':
            if not self._move_floor_robot_home.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Floor Robot node not running')
                return

            future = self._move_floor_robot_home.call_async(request)


        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Moved {robot_name} to home position')
        else:
            self.get_logger().warn(future.result().message)
        