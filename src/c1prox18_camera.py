import rospy
from geometry_msgs.msg import Twist
from serial_handler import SerialHandler

class C1ProX18:
    def __init__(self):
        # Subscriber for velocity changes for motor control
        self.vel_subscriber = rospy.Subscriber(
            "/cmd_vel", Twist, self.vel_callback, queue_size=10)
        
        # Subscriber for attribute changes for motor control
        # self.scf4_subscriber = rospy.Subscriber(
        #     "/scf4", Scf4, self.scf4_callback, queue_size=10)
        
        self.twist_last = Twist()
        self.serial_handler = SerialHandler()

        # TODO: move to settings.json
        self.port = ""
        self.baudrate = 115200
        self.timeout = -1

        self.motor_a = {
            "jog_steps": 50,
            "speed_min": 400,
            "speed_max": 4000,
            "vel_factor": 1000,
        }

        self.motor_b = {
            "jog_steps": 25,
            "speed_min": 400,
            "speed_max": 4000,
            "vel_factor": 1000,
        }


        # For image data check http://wiki.ros.org/Sensors/Cameras
        pass

    def _clamp(self, speed_new, motor_type):
        """Clamps the motor speed

        This method takes a speed value and a motor type, and clamps the
        speed between the min and max range depending on the motor type.
        It also converts the speed to an integer value.

        Args:
            speed_new (float): The speed value to clamp
            motor_type (str): The type of motor the speed depends on

        Returns:
            int: The clamped speed value converted to an integer
        """
        # Grabs the correct motor and clamps the speed within range
        motor = self.motor_a if motor_type == "A" else self.motor_b
        clamped = max(motor["speed_min"], min(motor["speed_max"], speed_new))

        return int(clamped)
    
    def _vel_callback_helper(self, twist, motor_type):
        """_summary_

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
            tuple(int|None, float|None): _description_
        """
        # Init Nones
        speed = None
        steps = None

        # Select the configuration for the right motor 
        motor = self.serial_handler.config[motor_type]

        if motor_type == "A":
            # If the motor type is A
            vel_curr = twist.linear.x
            vel_last = self.twist_last.linear.x
        else:
            # If the motor type is B
            vel_curr = twist.angular.z
            vel_last = self.twist_last.angular.z

        if vel_curr != 0:
            # Check the direction and make motor steps either + or -
            steps = motor["jog_steps"] * (1 if vel_curr > 0 else -1)

            if not abs(vel_curr) == abs(vel_last):
                # If speed is different from the last
                speed = vel_curr * motor["vel_factor"]
                
                # Inform about the speed change for a specific motor
                rospy.loginfo(f"Motor {motor_type} speed changed to {speed}")
        
        return speed, steps

    def vel_callback(self, twist):
        """_summary_

        Args:
            twist (Twist): The twist message from /cmd_vel topic
        """
        # Get the speed and steps values for motors A & B 
        speed_a, steps_a = self._vel_callback_helper(twist, "A")
        speed_b, steps_b = self._vel_callback_helper(twist, "B")

        # Send the values to the controller to execute cmd
        self.serial_handler.change_speed(speed_a, speed_b)
        self.serial_handler.move_motors(steps_a, steps_b)

        # Set last twist value
        self.twist_last = twist
        

    def publish(self):
        pass

    def connect(self):
        self.serial_handler.connect(self.port, self.baudrate, self.timeout)