import rospy
import multiprocessing

class ZoomTracker(multiprocessing.Process):
    def __init__(self, serial_handler, final_callback, min_static_dur=3):
        super().__init__()

        self.serial_handler = serial_handler
        self.final_callback = final_callback
        self.min_static_dur = min_static_dur

    def run(self):
        # Wait for the motors to fully stop
        self.serial_handler.await_idle("A")

        # Get the start time since idle
        start_time = rospy.Time().now()
        
        while True:
            # Calculate total time motors have been idle
            static_dur = rospy.Time().now() - start_time
            
            if rospy.Time.to_sec(static_dur) >= self.min_static_dur:
                # If min idle duration
                self.final_callback()
                break