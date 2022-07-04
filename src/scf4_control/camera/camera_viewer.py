import cv2
import rospy

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class CameraViewer:
    def __init__(self, width=1920, height=1080):
        # Video output dims
        self.width = width
        self.height = height

        # A communication bridge
        self.bridge = CvBridge()

        self.cam_subscriber = rospy.Subscriber(
            "/camera", CompressedImage, self.cam_callback, queue_size=1)
    
    def cam_callback(self, data):
        try:
            # Retrieve the compressed, image and resize it
            frame = self.bridge.compressed_imgmsg_to_cv2(data)
            frame_scaled = cv2.resize(frame, (self.width, self.height))

            # Create a resizable window and load the frame/image
            cv2.namedWindow("C1ProX18 Camera", cv2.WINDOW_NORMAL)
            cv2.imshow("C1ProX18 Camera", frame_scaled)
        except CvBridgeError as e:
            # Log the error
            rospy.logerr(e)

        if cv2.waitKey(1) & 0xFF == ord('q') or \
           cv2.getWindowProperty("C1ProX18 Camera", cv2.WND_PROP_VISIBLE) < 1:
            # If `Q` key or window exit button clicked 
            rospy.signal_shutdown("Quit button clicked")
        