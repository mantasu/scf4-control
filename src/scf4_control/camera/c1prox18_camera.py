import os
import cv2
import rospy
from sys import platform

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from scf4_control.msg import Scf4

from cv_bridge import CvBridge, CvBridgeError
from scf4_control.serial.serial_handler import SerialHandler
from scf4_control.utils import parse_json, verify_path, get_fourcc
from scf4_control.tracker.motor_tracker import MotorTracker

class C1ProX18:
    def __init__(self, config_path="config.json", is_relative=True):
        # Parse config.json from the given file path
        config = parse_json(config_path, is_relative)

        self._init_capture(config["capture"])
        self._init_writer(config["writer"])

        # Last speed vals to check changes
        self.speed_last = {"A": 0, "B": 0}
        

        # Create a serial handler to handle the G-code via serial port
        #self.serial_handler = SerialHandler(config["serial"], config["motors"])

        #self.motor_tracker = MotorTracker(self.capture, self.serial_handler, self.config["min_idle_time"])

        # Subscriber for velocity changes for motor control
        #self.vel_subscriber = rospy.Subscriber(
        #    "/cmd_vel", Twist, self.vel_callback, queue_size=10)
        
        # Subscriber for attribute changes for motor control
        self.scf4_subscriber = rospy.Subscriber(
            "/scf4", Scf4, self.scf4_callback, queue_size=10)

        # For image data check http://wiki.ros.org/Sensors/Cameras
        self.cam_publisher = rospy.Publisher(
            "/camera", CompressedImage, queue_size=1)
    
    
    def _init_capture(self, config):
        # Open CV bridge for ros msg
        self.cv_bridge = CvBridge()

        self.img_format = config["format"]

        if config["backend"] < 0:
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
            backend = config["backend"]

        # Initialize and open video capture device based on ID & API
        self.capture = cv2.VideoCapture(config["device_id"], backend)

        if self.capture.isOpened():
            # Log the capture device ID and the video capture API
            rospy.loginfo(f"Successfully opened capture device:"
                          f"\n\t* DEV: {config['device_id']}"
                          f"\n\t* API: {self.capture.getBackendName()}")
        else:
            # Log the error if ID or API of the capture device is wrong
            rospy.logerr(f"Invalid capture device '{config['device_id']}'"
                         f" or video capture API (backend): '{backend}'")

            # Throw an Open CV error if either device ID or API is bad
            raise cv2.error(f"Failed to init/open the capture device")
        
        # Acquire an actual FOURCC object based on the code
        fourcc = cv2.VideoWriter.fourcc(*config["fourcc"])

        # Set config properties for the capture device
        self.capture.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.capture.set(cv2.CAP_PROP_FPS, config["fps"])
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, config["width"])
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config["height"])
        
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

    def _init_writer(self, config):
        # Verify output directory and create it or nested dirs if needed
        out_dir = verify_path(config["out_dir"], config["is_relative"])

        # Callback to create an actual video writer
        self.make_writer = lambda: cv2.VideoWriter(
            os.path.join(out_dir, f"{rospy.get_time()}.{config['format']}"),
            cv2.VideoWriter.fourcc(*config["fourcc"]),
            config["fps"],
            (config["width"], config["height"]))
        
        # Log the directory at which the video recordings will be saved
        rospy.loginfo(f"Recordings (if any) will be saved at {out_dir}")
    
    def _vel_callback_helper(self, twist, motor_type):
        """A helper function to generate motor velocity values

        Takes a twist object and compares it with the last twist object.
        Any absolute value changes indicate motor speed changes and a
        corresponding speed value is created based on motor's 
        `vel_factor` attribute. The property `jog_steps` define the
        number of steps to move and it is simply multiplied by the sign
        of the twist velocity component that corresponds to a specified
        motor. For linear (motor A), only the `x` component is
        considered, for angular (motor B) - only the `z`*.

        *For speed changes, it is also possible to compute overall
        magnitude and direction of a 3D vector (for both linear and
        angular). This would make the speed changes invariant to the
        value changes in 3 axes (because there's only positive/negative
        direction (only 1 axis) for each motor), however this would
        involve more computations and reduce efficiency.  

        Note: for simplicity, ALL speed changes are captured, even the
            ones beyond min/max ranges. Every time the speed is changed 
            beyond the boundary, a corresponding command will be sent to
            a motor with the clamped value.

        Args:
            twist (Twist): The twist message from some topic
            motor_type (str): The type of motor - A (zoom) or B (focus)

        Returns:
            tuple(int, int): A tuple of speed and steps values for the
                specified motor. They can be `None`.
        """
        # Init Nones
        speed = None
        steps = None

        # Select the configuration for the right motor 
        motor = self.serial_handler.config[motor_type]

        # Get last and current speed values 
        vel_last = self.speed_last[motor_type]
        vel_curr = twist.linear.x if motor_type == "A" else twist.angular.z

        if vel_curr != 0:
            # Check the direction and make motor steps either + or -
            steps = motor["jog_steps"] * (1 if vel_curr > 0 else -1)

            if not abs(vel_curr) == abs(vel_last):
                # If current speed is different from the last
                speed = abs(vel_curr) * motor["vel_factor"]
                self.speed_last[motor_type] = abs(vel_curr)
                
                # Inform about the speed change for a specific motor
                rospy.loginfo(f"Motor {motor_type} speed changed to {speed}")

        return speed, steps

    def vel_callback(self, twist):
        """Coverts twist object to a G-code command and send it

        The twist object's linear velocity (component x) accounts for
        motor A (zoom) movement and angular velocity (component z) for
        motor B (focus) movement. Each component can be positive or
        negative indicating motor movement direction. If there are any
        changes in the absolute values, motor speed changes occur
        accordingly. See :func:`~C1ProX18._vel_callback_helper` for
        more details.

        Args:
            twist (Twist): The twist message from /cmd_vel topic
        """
        # Get the speed and steps values for motors A & B 
        speed_a, steps_a = self._vel_callback_helper(twist, "A")
        speed_b, steps_b = self._vel_callback_helper(twist, "B")

        # Send the values to the controller to execute cmd
        self.serial_handler.set_speed(speed_a, speed_b)
        self.serial_handler.move(steps_a, steps_b)

        # Set last twist value
        self.twist_last = twist

        self.reset_motion_tracker()
    
    def scf4_callback(self, scf4):
        if scf4.stop:
            self.serial_handler.stop()
        
        if scf4.filter_position >= 0:
            self.serial_handler.set_filter_position(scf4.filter_position)
        
        if scf4.coordinate_mode >= 0:
            self.serial_handler.set_coordinate_mode(scf4.coordinate_mode)
        
        if scf4.motor_move_mode >= 0:
            self.serial_handler.set_motor_move_mode(scf4.motor_move_mode)
        
        if len(scf4.steps) > 0:
            self.serial_handler.move(*scf4.steps)
        
        if len(scf4.speed) > 0:
            self.serial_handler.set_speed(*scf4.speed)

        if len(scf4.count) > 0:
            self.serial_handler.set_counter(*scf4.count)

        if scf4.command != "":
            self.serial_handler.send_command(scf4.command)
        
        if scf4.wait > 0:
            self.serial_handler.wait(scf4.wait)

    def publish(self):
        ret, frame = self.capture.read()

        if self.is_recording:
            self.write_video(frame)

        frame_compressed = self.cv_bridge.cv2_to_compressed_imgmsg(
            frame, dst_format=self.img_format)
        
        try:
            self.cam_publisher.publish(frame_compressed)
        except CvBridgeError as e:
            rospy.logerr(e)
    
    def write_video(self):
        if self.is_recording:
            rospy.logwarn("Recording is already in process.")
            return
        
        self.is_recording