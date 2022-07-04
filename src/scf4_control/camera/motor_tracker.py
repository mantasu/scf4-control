import rospy
import multiprocessing

class ZoomTracker(multiprocessing.Process):
    def __init__(self, success_callback, idle_callback, idle_duration):
        super().__init__()

        self.success_callback = success_callback
        self.idle_callback = idle_callback
        self.idle_duration = idle_duration

    def run(self):
        self.idle_callback()

        start_time = rospy.Time().now()
        
        while True:
            duration = rospy.Time().now() - start_time
            
            if duration >= self.idle_duration:
                self.success_callback()
                break

class MotorTracker:
    def __init__(self, serial_handler, min_idle_time):

        self.serial_handler = serial_handler
        self.min_idle_time = min_idle_time

        self.is_idle_zoom = False
        self.idle_zoom_pos = self.serial_handler.get_motor_position("A")
        self.await_idle_callback = lambda: self.serial_handler.await_idle("A")

        self.zoom_tracker = ZoomTracker(self._set_idle_zoom, 
            self.await_idle_callback , self.min_idle_time)

        self.zoom_tracker.start()
    
    def _set_idle_zoom(self, is_idle=True):
        self.is_idle_zoom = is_idle

        if is_idle:
            self.idle_zoom_pos = self.serial_handler.get_motor_position("A")    

    def reset_zoom_tracking(self):
        self.zoom_tracker.terminate()
        self._set_idle_zoom(False)
        
        self.zoom_tracker = ZoomTracker(self.is_idle_zoom, 
            self.await_idle_callback, self.min_idle_time)
        
        self.zoom_tracker.start()
