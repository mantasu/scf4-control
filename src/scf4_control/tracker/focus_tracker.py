import cv2
import multiprocessing

class FocusTracker(multiprocessing.Process):
    def __init__(self, capture, set_fm_callback, roi=None):
        super().__init__()
        
        self.capture = capture
        self.set_fm_callback = set_fm_callback
        
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
        ret, frame = self.capture.read()
        fm, roi = self.eval_focus(frame)

        self.set_fm_callback(fm)