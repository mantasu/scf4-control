import cv2
import rospy
import multiprocessing

from scf4_control.tracker.zoom_tracker import ZoomTracker
from scf4_control.tracker.focus_tracker import FocusTracker

class MotorTracker:
    def __init__(self, capture, serial_handler, min_static_dur):

        # Assign passed attributes
        self.capture = capture
        self.serial_handler = serial_handler
        self.min_static_dur = min_static_dur

        self.fm = None
        self.is_idle_zoom = False
        self.idle_zoom_pos = self.serial_handler.get_motor_position("A")

        self.zoom_tracker = ZoomTracker(self.serial_handler,
            self._set_idle_zoom, self.min_static_dur)
        
        self.focus_tracker = FocusTracker(self.capture,
            self._set_focal_measure)

        self.zoom_tracker.start()
    
    def _set_idle_zoom(self, is_idle=True):
        """Sets the idle status for the zoom motor

        Takes either `True` or `False` and updates this class' attribute
        which tells whether the zoom motor is idle (more precisely,
        whether it has been in idle for quite some time). If it is
        `True`, it also updates the idle motor position.

        Args:
            is_idle (bool, optional): Whether to update the zoom motor
                status to idle/stopped/static. Defaults to True.
        """
        # Update the class attribute
        self.is_idle_zoom = is_idle

        if is_idle:
            # If static, also update the position at which it is stopped
            self.idle_zoom_pos = self.serial_handler.get_motor_position("A")
            self.zoom_tracker.terminate()
            self.find_best_focus_pos()
    
    def _set_focal_measure(self, fm):
        self.fm = fm

    def reset_zoom_tracking(self):
        self.zoom_tracker.terminate()
        self._set_idle_zoom(False)
        
        self.zoom_tracker = ZoomTracker(self.serial_handler,
            self._set_idle_zoom, self.min_static_dur)
        
        self.zoom_tracker.start()
    
    def find_best_focus_pos(self):
        self.focus_tracker = FocusTracker(self.capture,
            self._set_focal_measure)
        
        self.focus_tracker.start()

        pos_min = self.serial_handler.config["B"]["count_min"]
        pos_max = self.serial_handler.config["B"]["count_max"]

        self.serial_handler.set_coordinate_mode(0)
        self.serial_handler.move(None, pos_min)
        self.serial_handler.await_idle("B")
        self.serial_handler.move(None, pos_max)
        
        best_fm = 0
        
        while self.serial_handler.is_moving("B"):
            if self.fm > best_fm:
                best_fm = self.fm
                self.best_focus_pos = self.serial_handler.get_motor_position("B")
        
        self.serial_handler.move(None, self.best_focus_pos)
        self.serial_handler.set_coordinate_mode(1)
