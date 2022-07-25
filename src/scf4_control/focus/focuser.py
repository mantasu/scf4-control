import rospy
import multiprocessing

from scf4_control.focus.sweeper import Sweeper

class Focuser(multiprocessing.Process):
    def __init__(self, streamer, serial, sleep_time=0.1):
        super().__init__()

        # Serial & sleep dur
        self.serial = serial
        self.sleep_time = sleep_time

        # Sweeper to adjust focus based on captured frames
        self.sweeper = Sweeper(streamer, serial, sleep_time)

        # Init the events to monitor updates
        self._event_stop = multiprocessing.Event()
        self._event_init = multiprocessing.Event()
        self._event_wait = multiprocessing.Event()
        self._event_focus = multiprocessing.Event()

        # Initialize wait start time
        self._wait_start_time = None
    
    def _clear_events(self):
        if self._event_wait.is_set():
            # Clear start time and event
            self._wait_start_time = None
            self._event_wait.clear()
        elif self._event_focus.is_set():
            # Clear the focusing events
            self.sweeper.clear_events()
            self._event_focus.clear()
            self._event_stop.clear()
    
    def _execute_init(self):
        if not self._event_init.is_set():
            return
        
        # Clear events, wait
        self._clear_events()
        self._event_wait.set()
        self._event_init.clear()
    
    def _execute_wait(self):
        """Executes waiting event

        Once the `wait event` is set, it waits for the zoom motor to
        stop moving. Once it stops, a countdown starts for the minimum
        duration the zoom motor should not move. If it does not move for
        the specified time, the `eval event` is set and this method
        always just returns until the `wait event` is triggered again.
        """
        if not self._event_wait.is_set():
            return
        elif self._wait_start_time is None and not self.serial.is_moving('A'):
            # Set countdown for min static zoom time
            self._wait_start_time = rospy.get_time()
        elif self._wait_start_time is not None and rospy.get_time() - \
             self._wait_start_time >= self.serial.config["min_idle_time"]:
            # Set focus, clear wait
            self._event_focus.set()
            self._event_wait.clear()
            self._wait_start_time = None
            rospy.loginfo(f"Zoom motor stopped. Focus motor started.")
        else:
            # If still moving, just wait
            rospy.sleep(self.sleep_time)
    
    def _execute_focus(self):
        self.sweeper.execute(self._event_stop)

    def reset(self):
        # Enable an init event
        self._event_init.set()
    
    def stop(self):
        # Set the stop event
        self._event_stop.set()

        if self.is_alive():
            # Wait to complete
            self.join()
    
    def run(self):
        """_summary_

        Note: event execute methods are surrounded with locks so that 
            an event switch would not happen after it is reset from
            another thread. Efficiency could be gained by surrounding
            with locks only those if/else blocks which include the
            change of the event, however it does not increase the speed
            notably and the current design is chosen for readability.
        """
        while True:
            if self._event_stop.is_set():
                # Clear remaining events
                self._event_stop.clear()
                self._event_focus.clear()
                break
            
            # Make focus updates
            self._execute_init()
            self._execute_wait()
            self._execute_focus()