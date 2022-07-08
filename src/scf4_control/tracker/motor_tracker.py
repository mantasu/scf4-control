from multiprocessing import Process
from scf4_control.tracker.zoom_tracker import ZoomTracker
from scf4_control.tracker.focus_tracker import FocusTracker

class MotorTracker:
    def __init__(self, streamer, serial_handler):

        # Set passed attributes
        self.streamer = streamer
        self.serial_handler = serial_handler

        # Init focus props
        self.fm_best = 0.0
        self.fm_curr = None
        self.focus_pos_best = None
        self.adjust_process = None

        # Initialize zoom and focus trackers to check when to focus lens
        self.zoom_tracker = ZoomTracker(self.serial_handler, self.idle_callback)
        self.focus_tracker = FocusTracker(self.streamer, self.fm_callback)

        # Start both processes
        self.zoom_tracker.start()
        self.focus_tracker.start()
    
    def idle_callback(self, is_idle=True):
        if is_idle:
            self.adjust_process = Process(target=self.adjust_focus)
            self.adjust_process.start()
    
    def fm_callback(self, fm):
        # Assign current fm
        self.fm_curr = fm
    
    def focus_pose_callback(self, pos):
        if self.fm_curr > self.fm_best:
            # If better fm, set it + pos
            self.fm_best = self.fm_curr
            self.focus_pos_best = pos
    
    def adjust_focus(self):
        # Sweep the focus motor until the best position is found and go
        self.focus_tracker.set_focusing()
        self.serial_handler.sweep_once("B", callback=self.focus_pose_callback)
        self.serial_handler.set_coordinate_mode(0)
        self.serial_handler.move(None, self.focus_pose_best)
        self.serial_handler.await_idle("B")
        self.serial_handler.set_coordinate_mode(1)

        # Reset focus args
        self.fm_best = 0
        self.focus_pos_best = None
    
    def release(self):
        if self.adjust_process is not None:
            self.adjust_process.terminate()
        
        self.zoom_tracker.set_stop()
        self.focus_tracker.set_stop()

        self.zoom_tracker.join()
        self.focus_tracker.join()
        
