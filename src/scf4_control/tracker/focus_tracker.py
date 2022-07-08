import cv2
import rospy
import multiprocessing

class FocusTracker(multiprocessing.Process):
    def __init__(self, streamer, set_fm, roi=None, sleep_time=0.3):
        super().__init__()
        
        # Set the attributes
        self.set_fm = set_fm
        self.streamer = streamer
        self.sleep_time = sleep_time
        
        if roi is not None:
            # Vals 2 coords [x, y, w, h] -> [x0, x1, y0, y1]
            self.roi = {"x0": roi[0], "x1": roi[0] + roi[2],
                        "y0": roi[1], "y1": roi[1] + roi[3]}
        else:
            # Otherwise keep
            self.roi = None

        # Init the events to monitor in the run method
        self._focusing_event = multiprocessing.Event()
        self._stop_event = multiprocessing.Event()
    
    def set_focusing(self, focus=True):
        if focus:
            # Set the focusing event
            self._focusing_event.set()
        else:
            # Unset the focusing event
            self._focusing_event.clear()
    
    def set_stop(self):
        # Set the stop event
        self._stop_event.set()
    
    def is_focusing_set(self):
        return self._focusing_event.is_set()

    def is_stop_set(self):
        return self._stop_event.is_set()
    
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
        while True:
            if self.is_stop_set():
                # Stop
                break

            if self.is_focusing_set():
                # Get frame from capture dev
                frame = self.streamer.read()

                if not frame:
                    # Send a warning in case the frame is of type None
                    rospy.logwarn("Missed frame while calculating FM.")
                    continue
                
                # Compute focal measure & send
                fm, _ = self.eval_focus(frame)
                self.set_fm(fm)
            else:
                # If there is nothing to do
                rospy.sleep(self.sleep_time)