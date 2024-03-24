

class CustomTimer():
    def __init__(self,node):
        
        self.node = node # node which will use this class
        self._wait_flag = False #is for waiting for another order within 15 sec
        self._delay_flag = False #for delay after order is recieved before task 6 (ship and submit)

        self._sleep_start_time = None

    def check_wait_flag(self):
        return not self._wait_flag and self.sleep()
    
    def check_delay_flag(self):
        return not self._delay_flag and self.sleep()
    
    def sleep(self):
        if self._sleep_start_time is None:
            self._sleep_start_time = self.node.get_clock().now()

        if (self.node.get_clock().now()-self._sleep_start_time).nanoseconds*1e-9 > 15:
            self._sleep_start_time = None
            self._wait_flag = not self._delay_flag
            self._delay_flag = not self._wait_flag
            return False
        return True
    
    def reset_flags(self):
        self._wait_flag = False
        self._delay_flag = False
