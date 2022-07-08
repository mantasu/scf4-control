import rospy
import multiprocessing

class ZoomTracker(multiprocessing.Process):
    def __init__(self, serial_handler, set_idle, sleep_time=0.3):
        super().__init__()

        # Assign the attributes
        self.set_idle = set_idle
        self.sleep_time = sleep_time
        self.serial_handler = serial_handler
        
        # Set idle parameters
        self._is_idle = False
        self._start_time = None

        # Init the events to monitor in the run method
        self._moving_event = multiprocessing.Event()
        self._stop_event = multiprocessing.Event()

    def set_moving(self, moved=True):
        if moved:
            # Set the moving event
            self._moving_event.set()
        else:
            # Unset the moving event
            self._moving_event.clear()

    def set_stop(self):
        # Set the stop event
        self._stop_event.set()
    
    def is_moving_set(self):
        return self._moving_event.is_set()

    def is_stop_set(self):
        return self._stop_event.is_set()
    
    def is_idle(self):
        return self._is_idle

    def run(self):
        while True:
            if self.is_stop_set():
                # Stop
                break
            
            if self.is_moving_set():
                # Set non-idle state
                self.set_idle(False)
                self._is_idle = False

                if self.serial_handler.is_moving("A"):
                    # If still moving, just wait
                    rospy.sleep(self.sleep_time)
                else:
                    # Set non-moving and timestamp
                    self.set_moving(moved=False)
                    self._start_time = rospy.get_time()
            else:
                if self.is_idle() or self._start_time is None:
                    rospy.sleep(self.sleep_time)
                    continue
                
                # Calculate total duration motor has been idle
                idle_dur = rospy.get_time() - self._start_time
                idle_dur = rospy.Time.to_sec(idle_dur)

                if idle_dur >= self.serial_handler.config["min_idle_time"]:
                    # If min idle reached
                    self.set_idle(True)
                    self._is_idle = True
                else:
                    # If min idle time not reached
                    rospy.sleep(self.sleep_time)
                    self._is_idle = False