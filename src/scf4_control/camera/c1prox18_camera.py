import cv2
import rospy

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from scf4_control.msg import Scf4

from cv_bridge import CvBridge, CvBridgeError
from scf4_control.serial.serial_handler import SerialHandler
from scf4_control.utils import parse_json

class C1ProX18:
    def __init__(self, config_path="config.json", is_relative=True):
        # Parse config.json from the given file path
        config = parse_json(config_path, is_relative)

        # Self config is only camera
        self.config = config["camera"]

        # Create a serial handler to handle the G-code via serial port
        self.serial_handler = SerialHandler(config["serial"], config["motors"])

        # Open video capture device (C1ProX18 camera) and log it
        self.capture = cv2.VideoCapture(self.config["device_id"])
        rospy.loginfo(f"Opened capture device: {self.capture.isOpened()}")

        # OpenCV bridge for messages
        self.cv_bridge = CvBridge()

        # Last speed vals to check changes
        self.speed_last = {"A": 0, "B": 0}

        # Subscriber for velocity changes for motor control
        self.vel_subscriber = rospy.Subscriber(
            "/cmd_vel", Twist, self.vel_callback, queue_size=10)
        
        # Subscriber for attribute changes for motor control
        self.scf4_subscriber = rospy.Subscriber(
            "/scf4", Scf4, self.scf4_callback, queue_size=10)

        # For image data check http://wiki.ros.org/Sensors/Cameras
        self.cam_publisher = rospy.Publisher(
            "/camera", CompressedImage, queue_size=1)
    
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

        frame_compressed = self.cv_bridge.cv2_to_compressed_imgmsg(
            frame, dst_format=self.config["format"])

        try:
            self.cam_publisher.publish(frame_compressed)
        except CvBridgeError as e:
            rospy.logerr(e)