class CustomTimer():
    def __init__(self, node):
        '''
        Initialize the CustomTimer object with a given ROS node.

        Args:
            node (rclpy.node.Node): The ROS node.
        '''
        self.node = node  # Store the ROS node
        self._wait_flag = False  # Flag for waiting for another order within 15 seconds
        self._delay_flag = False  # Flag for delay after order is received before task 6 (ship and submit)
        self._sleep_start_time = None  # Store the start time for sleep
        
    def check_wait_flag(self):
        '''
        Check if the wait flag is not set and if the sleep time has elapsed.

        Returns:
            bool: True if the wait flag is not set and the sleep time has elapsed, False otherwise.
        '''
        return not self._wait_flag and self.sleep()
    
    def check_delay_flag(self):
        '''
        Check if the delay flag is not set and if the sleep time has elapsed.

        Returns:
            bool: True if the delay flag is not set and the sleep time has elapsed, False otherwise.
        '''
        return not self._delay_flag and self.sleep()
    
    def sleep(self):
        '''
        Handle sleeping for 15 seconds.

        Returns:
            bool: True if still within the sleep period, False if sleep is over.
        '''
        if self._sleep_start_time is None:
            self._sleep_start_time = self.node.get_clock().now()  # Record the start time if not already set
        
        elapsed_time = (self.node.get_clock().now() - self._sleep_start_time).nanoseconds * 1e-9
        
        if elapsed_time > 15:
            self._sleep_start_time = None
            self._wait_flag = not self._delay_flag
            self._delay_flag = not self._wait_flag
            return False
        return True
    
    def reset_flags(self):
        '''
        Reset all flags and start time.
        '''
        self._wait_flag = False
        self._delay_flag = False
        self._sleep_start_time = None
