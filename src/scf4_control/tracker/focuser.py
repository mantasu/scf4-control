import cv2
import rospy
import threading

class Focuser():
    def __init__(self, streamer, serial, sleep_time=0.1):
        super().__init__()

        # Init streamer & serial
        self.streamer = streamer
        self.serial = serial
        
        # F measure and pos
        self.fm_best = None
        self.fp_best = None

        # Init the events to monitor updates
        self._event_stop = threading.Event()
        self._event_moved = threading.Event()
        self._event_focus = threading.Event()

        # Extra variables for updates
        self._focus_sweep_set = False
        self._idle_start_time = None
        self.sleep_time = sleep_time
        self.thread = None
        self.roi = None

        self.fms = []
        self.pos = []
    
    def _at_pos(self, motor='B', pos="min"):
        if pos in ["min", "max"]:
            # If the position is one of minimum or maximum counter
            pos = self.serial.config[motor][f"count_{pos}"]
        
        # Num to string
        pos = str(pos)

        return self.serial.is_equal(motor, status_group=0, vals=pos)
    
    def _wait_zoom(self):
        if self._event_moved.is_set():
            if self.serial.is_moving('A'):
                # If still moving, just wait
                rospy.sleep(self.sleep_time)
            else:
                # Set non-moving and time
                self._event_moved.clear()
                self._idle_start_time = rospy.get_time()
        else:
            if self._idle_start_time is None:
                return
            
            # Calculate the total duration zoom motor has been idle
            idle_elapsed = rospy.get_time() - self._idle_start_time

            if idle_elapsed >= self.serial.config["min_idle_time"]:
                # Set focusing from now
                self._event_focus.set()
                self._idle_start_time = None
                rospy.loginfo("Zoom motor stopped. Focus motor started.")
            else:
                # If min idle time not reached
                rospy.sleep(self.sleep_time)

    def _wait_focus(self):
        if self._event_focus.is_set():
            if self.serial.is_moving('B'):
                if self._focus_sweep_set:
                    # Get frame from capture dev
                    frame = self.streamer.read()

                    if frame is None:
                        # Send a warning in case the frame not existing
                        rospy.logwarn("Missed frame while calculating FM.")
                    else:
                        # Get motor's position and focal measure
                        fp = self.serial.get_motor_position('B')
                        fm, _ = self.eval_focus(frame)

                        if self.fm_best is None or fm > self.fm_best:
                            # Update the best
                            self.fm_best = fm
                            self.fp_best = fp
                else:
                    # Wait for the sweep min_pos
                    rospy.sleep(self.sleep_time)
            elif self._at_pos(pos="min") and not self._focus_sweep_set:
                # Start moving focus motor from min pos to max
                pos_max = self.serial.config['B']["count_max"]
                speed_max = self.serial.config['B']["speed_max"]
                
                self.serial.set_speed(None, speed_max)
                self.serial.move(None, pos_max)

                self._focus_sweep_set = True
            elif self._at_pos(pos="max") and self._focus_sweep_set:
                # Clear the focus event
                self._event_focus.clear()
                self._focus_sweep_set = False
            else:
                # Start moving focus motor from curr pos to min
                pos_min = self.serial.config['B']["count_min"]
                self.serial.move(None, pos_min)
    
    def _wait_adjust(self):
        if not (self._event_moved.is_set() or self._event_focus.is_set()):
            if self.serial.is_moving('B'):
                # Just sleep if it is moving
                rospy.sleep(self.sleep_time)
            elif not self._at_pos(pos=self.fp_best):
                # Set the mode to absolute and move
                self.serial.set_coordinate_mode(0)
                self.serial.move(None, self.fp_best)
            else:
                # Set the mode to relative and stop
                self.serial.set_coordinate_mode(1)
                self._event_stop.set()

    def start(self):
        # Set moved/clear focus 
        self._event_moved.set()
        self._event_focus.clear()

        # Clear best values
        self.fm_best = None
        self.fp_best = None

        if self.thread is None or not self.thread.is_alive():
            # If thread is None or isn't running, start running
            self.thread = threading.Thread(target=self.update)
            self.thread.start()
    
    def stop(self):
        # Set the stop event
        self._event_stop.set()

        if self.thread is not None and self.thread.is_alive():
            # Wait to complete
            self.thread.join()
    
    def eval_focus(self, img):
        if self.roi is not None:
            # Extract ROI if the coordinates exist
            roi = img[self.roi["y0"]:self.roi["y1"],
                      self.roi["x0"]:self.roi["x1"]]
        else:
            # All img
            roi = img
        
        # Get the grayscale ROI & calculate focal measure
        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        fm = cv2.Laplacian(roi_gray, cv2.CV_64F).var()

        return fm, roi
    
    def update(self):
        while True:
            if self._event_stop.is_set():
                # If stop event was set
                self._event_stop.clear()
                break
            
            # Monitor updates
            self._wait_zoom()
            self._wait_focus()
            self._wait_adjust()