import cv2
import rospy

from sys import platform
from scf4_control.utils import get_fourcc

class Capturer():
    def __init__(self, config):
        # The compression format via bridge
        self.dst_format = config["format"]

        # Get the video capture API backend based on OS
        backend = self._get_backend(config["backend"])

        # Initialize and open video capture device based on ID & API
        self.capture = cv2.VideoCapture(config["device_id"], backend)
        self._verify_capture_opened(config["device_id"], backend)
        
        # Acquire an actual FOURCC object based on the code
        fourcc = cv2.VideoWriter.fourcc(*config["fourcc"])

        # Set config properties for the capture device
        self.capture.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.capture.set(cv2.CAP_PROP_FPS, config["fps"])
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, config["width"])
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config["height"])
        
        # Log the properties that the video capturer assigned to itself
        fps, width, height, fourcc = self._verify_capture_properties()

        # Assign props
        self.fps = fps
        self.width = width
        self.height = height
        self.fourcc = fourcc
        self.delay = config["delay"]
    
    def _get_backend(self, desired_backend):
        if desired_backend < 0:
            if platform == "linux":
                # V4L2 for the Linux OS
                backend = cv2.CAP_V4L2
            elif platform == "darwin":
                # AvFoundation for the MAC iOS
                backend = cv2.CAP_AVFOUNDATION
            elif platform == "win32":
                # DirectShow for win32
                backend = cv2.CAP_DSHOW
            else:
                # Otherwise pick auto
                backend = cv2.CAP_ANY
        else:
            # Otherwise enum is provided
            backend = desired_backend
        
        return backend

    def _verify_capture_opened(self, device_id, backend):
        if self.capture.isOpened():
            # Log the capture device ID and the video capture API
            rospy.loginfo(f"Successfully opened capture device:"
                          f"\n\t* DEV: {device_id}"
                          f"\n\t* API: {self.capture.getBackendName()}")
        else:
            # Log the error if ID or API of the device is wrong
            rospy.logerr(f"Invalid capture device '{device_id}'"
                         f" or video capture API (backend): '{backend}'")

            # Throw an Open CV error if either device ID or API is bad
            raise cv2.error(f"Failed to init/open the capture device")

    def _verify_capture_properties(self):
        # Check the actual properties for camera
        fps = self.capture.get(cv2.CAP_PROP_FPS)
        width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc_name = get_fourcc(int(self.capture.get(cv2.CAP_PROP_FOURCC)))

        # Log the assigned capture device properties
        rospy.loginfo(f"Capture device properties:"
                      f"\n\t* 4CC: {fourcc_name}"
                      f"\n\t* FPS: {fps}"
                      f"\n\t* RES: {width}x{height}")
        
        return fps, width, height, fourcc_name

    def read(self):        
        return self.capture.read()
    
    def release(self):
        # Release the capturer
        self.capture.release()