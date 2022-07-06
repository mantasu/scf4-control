import cv2
import rospy
import multiprocessing

class FocusTracker(multiprocessing.Process):
    def __init__(self, fm_callback, get_img_callback, roi=None):
        super().__init__()
        
        self.get_img_callback = get_img_callback
        self.fm_callback = fm_callback
        
        if roi is not None:
            # ROI values to coordinates [x, y, w, h] -> [x0, x1, y0, y1]
            self.roi = {
                "x0": roi[0],
                "x1": roi[0] + roi[2],
                "y0": roi[1],
                "y1": roi[1] + roi[3],
            }
        else:
            self.roi = None
    
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
    
    def run(self):
        ret, frame = self.get_img_callback()
        fm, roi = self.eval_focus(frame)

        self.fm_callback(fm)

class ZoomTracker(multiprocessing.Process):
    def __init__(self, on_success_callback, await_callback, min_idle_time=3):
        super().__init__()

        self.on_success_callback = on_success_callback
        self.await_callback = await_callback
        self.min_idle_time = min_idle_time

    def run(self):
        # Wait motors to stop
        self.await_callback()

        # Get the start time since idle
        start_time = rospy.Time().now()
        
        while True:
            # Calculate the total time the motors have been idle
            idle_time_elapsed = rospy.Time().now() - start_time
            
            if idle_time_elapsed >= self.min_idle_time:
                # If minimum idle duration
                self.on_success_callback()
                break

class MotorTracker:
    def __init__(self, capture, serial_handler, min_idle_time):

        self.capture = capture
        self.serial_handler = serial_handler
        self.min_idle_time = min_idle_time

        self.fm = None
        self.is_idle_zoom = False
        self.idle_zoom_pos = self.serial_handler.get_motor_position("A")
        self.await_idle_callback = lambda: self.serial_handler.await_idle("A")
        self.get_img_callback = lambda: self.capture.read()[1]

        self.zoom_tracker = ZoomTracker(self._set_idle_zoom, 
            self.await_idle_callback, self.min_idle_time)
        
        self.focus_tracker = FocusTracker(self._set_focal_measure,
            self.get_img_callback)

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
    
    def _set_focal_measure(self, fm):
        self.fm = fm

    def reset_zoom_tracking(self):
        self.zoom_tracker.terminate()
        self._set_idle_zoom(False)
        
        self.zoom_tracker = ZoomTracker(self.is_idle_zoom, 
            self.await_idle_callback, self.min_idle_time)
        
        self.zoom_tracker.start()
    
    def find_best_focus_pos():
        pass
