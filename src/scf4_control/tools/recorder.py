import os
import cv2
import rospy

from scf4_control.utils import verify_path, get_str_datetime

class Recorder():
    def __init__(self, config):
        # Verify output directory and create it or nested dirs if needed
        out_dir = verify_path(config["out_dir"], config["is_relative"])

        # Callback to create an actual video writer
        self.make_writer = lambda: cv2.VideoWriter(
            os.path.join(out_dir, f"{get_str_datetime()}.{config['format']}"),
            cv2.VideoWriter.fourcc(*config["fourcc"]),
            config["fps"],
            (config["width"], config["height"]))
        
        # Log the directory at which the video recordings will be saved
        rospy.loginfo(f"Recordings (if any) will be saved at {out_dir}")

        # Init attributes
        self.writer = None
        self.is_recording = False
        self.start_timestamp = None
        self.writer_duration = None
    
    def start_recording(self, duration=None):
        if self.is_recording:
            # If the recorder is already in process of writing
            rospy.logwarn("Recording is already in process.")
            return
        
        # Initialize recorder attributes
        self.writer = self.make_writer()
        self.writer_duration = duration
        self.start_time = rospy.get_time()
        self.is_recording = True

        # Create duration string and log that the recording has started
        duration_str = f" for {duration} seconds" if duration else "" 
        rospy.loginfo(f"Recording started{duration_str}.")
    
    def end_recording(self):
        if self.is_recording:
            # Release writer
            self.release()
            self.start_time = None
            self.writer_duration = 0
            self.is_recording = False

            # Log that the recording has ended
            rospy.loginfo("Recording finished.")
        else:
            # If recording hasn't been started yet
            rospy.loginfo("Start recording first.")
    
    def write_video(self, frame):
        if self.writer_duration is not None and self.writer_duration != 0 and \
           rospy.get_time() - self.start_time > self.writer_duration:
            # If duration reached
            self.end_recording()
        else:
            # Otherwise write frame
            self.writer.write(frame)

    def release(self):
        if self.writer is not None:
            # Release and set None
            self.writer.release()
            self.writer = None

