import cv2
import rospy
import threading
import numpy as np

class Focuser():
    def __init__(self, streamer, serial, sleep_time=0.1):
        super().__init__()

        # Init streamer & serial
        self.streamer = streamer
        self.serial = serial

        # Set sleep time & init lock
        self.sleep_time = sleep_time
        self.write_lock = threading.Lock()

        # Init thread, roi
        self.thread = None
        self.roi = None

        # Init the events to monitor updates
        self._event_stop = threading.Event()
        self._event_wait = threading.Event()
        self._event_prep = threading.Event()
        self._event_eval = threading.Event()
        self._event_move = threading.Event()

        # Init start times, best pos
        self._wait_start_time = None
        self._eval_start_time = None
        self._best_focus_pose = None

        # Init arrays
        self.fms = []
        self.pos = []
    
    def _clear_events(self):
        """
        
        Note: only one event can be set at a time (thus else-if
            statements), otherwise there's no point in having locks.
        """
        with self.write_lock:
            if self._event_wait.is_set():
                # Clear start time and event
                self._wait_start_time = None
                self._event_wait.clear()
            elif self._event_prep.is_set():
                # Reset the coordinate mode, speed
                self.serial.set_coordinate_mode(1)
                self.serial.set_speed(None, "def")

                # Clear the single event
                self._event_prep.clear()
            elif self._event_eval.is_set():
                # Reset the coordinate mode, speed
                self.serial.set_coordinate_mode(1)
                self.serial.set_speed(None, "def")

                # Re-init arrays
                self.fms.clear()
                self.pos.clear()

                # Clear start time and event
                self._eval_start_time = None
                self._event_eval.clear()
            elif self._event_move.is_set():
                # Reset the coordinate mode, speed
                self.serial.set_coordinate_mode(1)
                self.serial.set_speed(None, "def")

                # Clear focus pose and event
                self._best_focus_pose = None
                self._event_move.clear()

    def _time_met(self, event_type="wait"):
        if event_type == "wait":
            # Wait event start time & duration
            start_time = self._wait_start_time
            dur = self.serial.config["min_idle_time"]
        elif event_type == "eval":
            # Eval event start time & duration
            start_time = self._eval_start_time
            dur = self.streamer.capturer.delay

        return start_time is not None and rospy.get_time() - start_time >= dur
    
    def _at_pos(self, motor='B', pos="min"):
        if pos in ["min", "max"]:
            # If motor position counter is one of [min|max]
            pos = self.serial.config[motor][f"count_{pos}"]
        
        # Num to string
        pos = str(pos)

        return self.serial.is_equal(motor, status_group=0, vals=pos)
    
    def _update_pos(self):
        # Get focus motor position & add to list
        fp = self.serial.get_motor_position('B')
        self.pos.append(fp)

    def _update_fms(self):
        # Get frame from capture dev
        frame = self.streamer.read()

        if frame is None:
            # Send a warning in case the frame is not existing
            rospy.logwarn("Missed frame while calculating FM.")
        else:
            # Get focal measure and append
            fm, _ = self.eval_focus(frame)
            self.fms.append(fm)
    
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
        elif self._time_met(event_type="wait"):
            # Set prep, clear wait
            self._event_prep.set()
            self._event_wait.clear()
            self._wait_start_time = None
            rospy.loginfo("Zoom motor stopped. Focus motor started.")
        else:
            # If still moving, just wait
            rospy.sleep(self.sleep_time)  

    def _execute_prep(self):
        if not self._event_prep.is_set():
            return
        elif self.serial.is_moving('B'):
            # If still moving, just wait
            rospy.sleep(self.sleep_time)
        elif self._at_pos(pos="min"):
            # Set low speed and go to max pose
            self.serial.set_speed(None, "max")
            self.serial.move(None, "max")
            
            # Set the start time for the eval method
            self._eval_start_time = rospy.get_time()

            # Set eval, clear prep
            self._event_eval.set()
            self._event_prep.clear()
        else:
            # Set absolute mode, move to start
            self.serial.set_coordinate_mode(0)
            self.serial.set_speed(None, "min")
            self.serial.move(None, "min")

    def _execute_eval(self):
        if not self._event_eval.is_set():
            return
        elif self._time_met(event_type="eval") and self.serial.is_moving('B'):
            # Motor pos and FM
            self._update_pos()
            self._update_fms()
        elif self.serial.is_moving('B'):
            # Update focus pos
            self._update_pos()
        elif len(self.pos) > len(self.fms):
            # Calculate the FM
            self._update_fms()
        else:
            # Get the best focus position
            pos_idx = np.argmax(self.fms)
            self._best_focus_pose = self.pos[pos_idx]

            # Reset the list
            self.fms.clear()
            self.pos.clear()

            # Set move, clear eval
            self._event_move.set()
            self._event_eval.clear()
    
    def _execute_move(self):
        if not self._event_move.is_set():
            return
        elif self.serial.is_moving('B'):
            # If still moving, just wait
            rospy.sleep(self.sleep_time)
        elif self._at_pos(pos=self._best_focus_pose):
            # Reset the coordinate mode, speed
            self.serial.set_coordinate_mode(1)
            self.serial.set_speed(None, "def")

            # Set stop, clear move
            self._event_stop.set()
            self._event_move.clear()

            rospy.loginfo("Focus finished.")
        else:
            # Move focus motor to where the largest FM is
            self.serial.move(None, self._best_focus_pose)

    def start(self):
        # Prepare focus events
        self._clear_events()
        self._event_wait.set()

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
                # Clear the single event
                self._event_stop.clear()
                break
            
            with self.write_lock:
                # Make focus updates
                self._execute_wait()
                self._execute_prep()
                self._execute_eval()
                self._execute_move()
