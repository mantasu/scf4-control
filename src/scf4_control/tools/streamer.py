import rospy

from cv_bridge import CvBridge
from threading import Thread, Lock
from scf4_control.tools.capturer import Capturer
from scf4_control.tools.recorder import Recorder


class Streamer():
    """_summary_

    https://gist.github.com/allskyee/7749b9318e914ca45eb0a1000a81bf56
    """
    def __init__(self, capturer_config, recorder_config):
        # Initialize capture and recorder objects
        self.capturer = Capturer(capturer_config)
        recorder_config = self._verify_recorder_config(recorder_config)
        self.recorder = Recorder(recorder_config)

        # Open CV bridge for rosmsg
        self.cv_bridge = CvBridge()

        # Read and set the first captured camera frame
        self.grabbed, self.frame = self.capturer.read()
        self.started = False
        self.read_lock = Lock()
    
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

    def start(self) :
        if self.started :
            # If already started, log a warning message
            rospy.logwarn("Streaming already started")
            return None
        
        # Begin the thread
        self.started = True
        self.thread = Thread(target=self.update)
        self.thread.start()

        return self

    def update(self):
        while self.started:
            # Read the current frame from capturer
            grabbed, frame = self.capturer.read() 
            
            # Safely acquire a frame
            self.read_lock.acquire()
            self.grabbed, self.frame = grabbed, frame
            self.read_lock.release()

            if not grabbed:
                # If no response received, log a warning message
                rospy.logwarn("Could not capture camera frame.") 
                continue

            if self.recorder.is_recording:
                # Use a recorder to write the current frame
                self.recorder.write_video(self.frame)
            else:
                self.recorder.release()

    def read(self, return_msg=False):
        # Safely acquire a frame
        self.read_lock.acquire()
        frame = self.frame.copy()
        self.read_lock.release()

        if return_msg:
            # Also compute a compressed frame message for the ROS nodes
            frame_compressed_msg = self.cv_bridge.cv2_to_compressed_imgmsg(
                frame, dst_format=self.capturer.dst_format)
            
            return frame, frame_compressed_msg

        return frame
    
    def start_recording(self, duration=None):
        # Start to record (optionally for dur)
        self.recorder.start_recording(duration)
    
    def end_recording(self):
        # End the recording forcefully
        self.recorder.end_recording()

    def stop(self) :
        # Wait to terminate
        self.started = False
        self.thread.join()

    def __exit__(self, exc_type, exc_value, traceback):
        # Release the resources
        self.capturer.release()
        self.recorder.release()