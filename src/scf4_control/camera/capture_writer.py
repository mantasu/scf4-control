import os
import cv2
import rospkg

class CaptureWriter:
    def __init__(self, **kwargs):
        self.out_dir = self.config["out_dir"]

        if self.config["is_relative"]:
            rel_path = rospkg.RosPack().get_path("scf4_control")
            self.out_dir = os.path.join(rel_path, self.config["out_dir"])
        else:
            self.out_dir = self.config["out_dir"]

        self.writer = cv2.VideoWriter()

