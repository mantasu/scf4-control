import cv2
import rospy
import numpy as np
import multiprocessing

class Sweeper:
    
    SWEEP_SPEED_FAST = 4000

    def __init__(self, streamer, serial, sleep_time=0.1):
        
        self.serial = serial
        self.streamer = streamer
        self.sleep_time = sleep_time

        self.adjust_pos = "max"
        self.target_pos = "min"
        self.sweep_start_time = None
        self.sweep_speed = self.SWEEP_SPEED_FAST

        self.roi = None

        # Init arrays
        self.fms = []
        self.pos = []

        self._event_adjust = multiprocessing.Event()
        self._event_sweep = multiprocessing.Event()
    
    def _update_pos(self):
        # Get focus motor position & add to list
        fp = self.serial.get_motor_position('B')
        self.pos.append(fp)
    
    def _update_fms(self):
        if self.sweep_start_time is None or \
           rospy.get_time() - self.sweep_start_time < \
           self.streamer.capturer.delay:
            return

        # Get frame from capture dev
        frame = self.streamer.read()

        if frame is None:
            # Send a warning in case the frame is not existing
            rospy.logwarn("Missed frame while calculating FM.")
        else:
            # Get focal measure and append
            fm, _ = self.eval_focus(frame)
            self.fms.append(fm)
    
    def _execute_adjust(self, event_stop):
        if not self._event_adjust.is_set():
            return
        elif self.serial.is_moving('B'):
            # If still moving, just wait
            rospy.sleep(self.sleep_time)
        elif self.serial.is_at(b=self.adjust_pos):
            if self.target_pos is None:
                # Reset the coordinate mode, speed
                self.serial.set_coordinate_mode(1)
                self.serial.set_speed(None, "def")

                # Stop the fucus
                event_stop.set()
                self._event_adjust.clear()

                # Log that focusing has finished
                rospy.loginfo("Focus finished.")
            else:
                # Set sweep, end adjust
                self._event_sweep.set()
                self._event_adjust.clear()

                # Set low speed and go to max pose
                self.serial.set_speed(None, self.sweep_speed)
                self.serial.move(None, self.target_pos)
                
                # Set the start time for the eval method
                self.sweep_start_time = rospy.get_time()
                self.target_pos = None
        else:
            # Set absolute mode, adjust a pose
            self.serial.set_coordinate_mode(0)
            self.serial.set_speed(None, "min")
            self.serial.move(None, self.adjust_pos)
    
    def _execute_sweep(self):
        if not self._event_sweep.is_set():
            return
        elif self.serial.is_moving('B'):
            # Motor pos and FM
            self._update_pos()
            self._update_fms()
        elif len(self.pos) > len(self.fms):
            # Calculate the FM
            self._update_fms()
        elif len(self.fms) > 0:
            # Get the best focus position
            pos_idx = np.argmax(self.fms)
            self.adjust_pos = self.pos[pos_idx]

            self._eval_start_time = None

            # Reset the list
            self.fms.clear()
            self.pos.clear()

            if isinstance(self.sweep_speed, str) and self.sweep_speed == "max":
                # Reset fast/quick (initial) sweep speed
                self.sweep_speed = self.SWEEP_SPEED_FAST
            else:
                # Set up a precise sweep
                self.sweep_speed = "max"
                self.target_pos = self.adjust_pos + 1000
                self.adjust_pos = self.adjust_pos - 1000
            
            # Set adjust, clear eval
            self._event_adjust.set()
            self._event_sweep.clear()
    
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
        fm = cv2.Laplacian(roi_gray, cv2.CV_32F).var()

        return fm, roi
    
    def clear_events(self):
        # Reset the coordinate mode, speed
        self.serial.set_coordinate_mode(1)
        self.serial.set_speed(None, "def")

        # Reset adjust & target
        self.adjust_pos = "max"
        self.target_pos = "min"

        if self._event_sweep.is_set():
            # Re-init arrays
            self.fms.clear()
            self.pos.clear()
            
            # Clear the sweep start time 
            self.sweep_start_time = None
    
    def execute(self, stop_event):
        self._execute_adjust(stop_event)
        self._execute_sweep()

