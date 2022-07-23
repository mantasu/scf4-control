import rospy
import ctypes
import numpy as np
import multiprocessing

from scf4_control.tools.capturer import Capturer
from scf4_control.tools.recorder import Recorder

class Streamer(multiprocessing.Process):
    """_summary_

    https://gist.github.com/allskyee/7749b9318e914ca45eb0a1000a81bf56
    """
    def __init__(self, capturer_config, recorder_config):
        # Initialize capture and recorder objects
        self.capturer = Capturer(capturer_config)
        recorder_config = self._verify_recorder_config(recorder_config)
        self.recorder = Recorder(recorder_config)

        self.recording_duration = multiprocessing.Value('i', 0)

        self.frame_shape = (self.capturer.height, self.capturer.width, 3)
        self.frame_size = int(np.prod(self.frame_shape))

        self.frame = multiprocessing.Array(ctypes.c_float, self.frame_size)
        self.frame = np.frombuffer(self.frame.get_obj(), dtype=np.float32)
        self.frame = self.frame.reshape(self.frame_shape)

        self._event_recording_start = multiprocessing.Event()
        self._event_recording_stop = multiprocessing.Event()
    
    def _verify_recorder_config(self, config):
        # Get FPS and resolution for capturer and recorder
        cap_fps, rec_fps = self.capturer.fps, config["fps"]
        cap_width, rec_width = self.capturer.width, config["width"]
        cap_height, rec_height = self.capturer.height, config["height"]

        if rec_fps > cap_fps:
            # Log a warning if the FPS for the recorder is too high
            rospy.logwarn(f"Recorder FPS too high compared to capture device. "
                          f"Setting recorder FPS from {rec_fps} to {cap_fps}.")
            
            # Set FPS to match cap
            config["fps"] = cap_fps
        
        if rec_width > cap_width or rec_height > cap_height:
            # Log a warning if the resolution for recorder is too high
            rospy.logwarn(f"Recorder resolution too high compared to capture "
                          f"device. Setting from {rec_width}x{rec_height} to "
                          f"{cap_width}x{cap_height}.")

            # Set config resolution lower
            config["width"] = cap_width
            config["height"] = cap_height
        
        return config
    
    def _handle_recording(self, frame):
        """Handles recording events and writes frames to a file

        Checks if start/stop recording events are set and starts/stops
        recording captured frames to a video file; clears events. It
        only asks the recorder to save the provided frame if the
        recorder is on.

        Args:
            frame (np.ndarray): The image to record as a video frame
        """
        if self._event_recording_start.is_set():
            # Start recording for the specified duration (or none)
            self.recorder.start_recording(self.recording_duration)
            self._event_recording_start.clear()

        if self._event_recording_stop.is_set():
            # End recording & clear event
            self.recorder.end_recording()
            self._event_recording_stop.clear()
        
        if self.recorder.is_recording:
            # Tell recorder to write a frame
            self.recorder.write_video(frame)
    
    def update_once(self):
        # Read the current frame from capturer
        grabbed, frame = self.capturer.read()
        
        if not grabbed or frame is None:
            # If no response received, log a warning message
            rospy.logwarn("Could not capture camera frame.")
            return
        
        # Record, update shared frame
        self._handle_recording(frame)
        np.copyto(self.frame, frame)

    def run(self):
        while True:
            if self._event_stop.is_set():
                # Stop event set - break
                self._event_stop.clear()
                break
            
            # Update each time
            self.update_once()

    def read(self):
        return self.frame.copy()
    
    def start_recording(self, duration=0):
        # Start to record (optionally for dur)
        self.recording_duration = duration
        self._event_recording_start.set()
    
    def end_recording(self):
        # End the recording forcefully
        self._event_recording_stop.set()

    def stop(self) :
        if self.is_running():
            # Wait till terminates
            self._event_stop.set()
            self.join()

    def __exit__(self, exc_type, exc_value, traceback):
        # Release the resources
        self.capturer.release()
        self.recorder.release()