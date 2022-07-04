import rospy
from serial_base import SerialBase
from scf4_control.utils import adc_to_volt

class SerialHandler(SerialBase):
    SWITCH_POSITION = 32767

    def __init__(self, serial_config, motors_config):
        """Initializes the serial handler

        Simply assigns a configuration dictionary as an attribute and
        sets the serial object to `None` (until it's connected).

        Note: for more information about configuration key-values
            check the README.md file.

        Args:
            serial_config (dict): The config with serial information
            motors_config (dict): The config with motor information
        """
        super().__init__(serial_config, motors_config)
        
        self.connect()
        self._init_controller()
        self._init_motors()

        self.static_timestamp = rospy.Time().now()
        self.static_duration = 0
        self.static_tracker = True

        # Assign the firmware version and the serial voltage params 
        self.version = self.get_version().split(', ')
        self.voltage = adc_to_volt(self.get_voltage())

        # Log information about version and voltage of the SCF4 module
        rospy.loginfo("Version:\n\t* " + "\n\t* ".join(self.version))
        rospy.loginfo("Voltage: " + (f"{self.voltage}" if self.voltage else "N/A"))
    
    def _init_controller(self):
        """Initializes the motor controller and its driver

        The initialization process (G-code commands):
            1. Reset and initialize motor driver
            2. Set stepping (should be 64 by default)
            3. Set normal move
            4. Switch to relative coordinate programming mode
            5. Energize PI LED
            6. Set motors and IR filter driver current
            7. Set motor idle current
            8. Set PI low/high detection voltage
        """
        # Inform the controller initialization started
        rospy.loginfo("Initializing controller... \r")

        # Initialize the motor controller via G-code commands
        self.send_command("$B2")
        self.send_command("M243 C6")
        self.send_command("M230")
        self.send_command("G91")
        self.send_command("M238")
        self.send_command("M234 A190 B190 C190 D90")
        self.send_command("M235 A120 B120 C120")
        self.send_command("M232 A400 B400 C400 E700 F700 G700")

        # Done initialization
        rospy.loginfo("Done")
    
    def _init_motors(self):
        
        # Inform that motor initialization started
        rospy.loginfo("Initializing motors... \r")

        status = self.get_status().split(", ")
        steps = [
            100 * (-1 if status[3] == "1" else 1),
            100 * (-1 if status[4] == "1" else 1),
            100 * (-1 if status[5] == "1" else 1),
        ]

        self.set_coordinate_mode(1)
        self.set_motor_move_mode(1)
        self.move(steps[:len(self.motors)])
        self.await_idle()

        self.set_motor_move_mode(0)
        self.move(*[-200]*len(self.motors))
        self.await_idle()

        self.set_motor_move_mode(1)
        self.move(*[100]*len(self.motors))
        self.await_idle()

        self.set_counter(*[self.SWITCH_POSITION]*len(self.motors))
        self.set_motor_move_mode(0)
        self.set_coordinate_mode(0)

        self.move(self.config["A"]["count_max"])
        self.await_idle()
        self.set_coordinate_mode(1)

        # Done initialization
        rospy.loginfo("Done")

    def is_moving(self, *args):
        """Checks if any of the motors are moving

        Gets the status for the motors and checks the last 3 values -
        one for every motor. `0` indicates not moving and `1` indicates
        a motor is moving. `False` is returned only if the specified
        motor(-s) or, if not specified, none of the motors, are moving.

        Args:
            *args: The type of motors to check if they're moving. For
                example 'A', 'C'. If nothing provided, all motors are
                checked

        Returns:
            bool: Whether the motors are currently moving
        """
        # Init the status
        is_moving = False
        status = self.get_status().split(", ")[-3:]

        if len(args) == 0:
            # Check all motors
            return "1" in status

        for motor_type in args:
            # Update with specific motor type
            status_idx = ord(motor_type) - 65
            is_moving = is_moving or (status[status_idx] == "1")

        return is_moving

    def await_idle(self, *args, timeout=5, on_timeout="raise"):
        """Halts till motors stop moving

        Enters an infinite `while` loop until the controller returns the
        motor status as "not moving" (for the specified motor(-s) or, if
        not specified, for every motor).

        Args:
            *args: The type of motors to check if they're moving. For
                example 'A', 'C'. If nothing provided, all motors are
                checked
            timeout (int, optional): The time to wait (in seconds)
                before forcefully stopping to wait. Defaults to 5.
            on_timeout (str, optional): The action to do if a timeout
                occurs - raise error or just log. Defaults to "raise".

        Raises:
            RuntimeError: If the timeout has been reached
        """
        # Check the rospy  start time
        start_time = rospy.Time.now()

        while True:
            if not self.is_moving(*args):
                # If motors have stopped
                break
            
            if rospy.Time.now() - start_time > timeout:
                if on_timeout == "raise":
                    # If the runtime error should be raised on timeout
                    raise RuntimeError("Waited too long for motors to stop")
                elif on_timeout == "log":
                    # If the timeout  error ony needs to be logged out
                    rospy.logerr("Waited too long for motors to stop")
                    break
            
            # Sleep 0.3 sec
            rospy.sleep(.1)

    def get_motor_position(self, *args, return_single_in_list=False):
        """Gets the motor position(-s)

        For every desired motor type, it checks its position as given by
        the SCF4 controller status, converts it to integer and returns.
        Single motor position by default is returned not in an array but
        as a single value.

        Args:
            *args: The motor types to check, e.g., 'A', 'C'
            return_single_in_list (bool, optional): Whether to return a
                single value in a list. Defaults to False.

        Returns:
            list|int: A list of motor positions as given by SCF4
                controller response. Or single value if only 1 motor
        """
        status = self.get_status().split(", ")[:3]
        positions = [int(status[ord(motor_type) - 65]) for motor_type in args]

        if len(positions) == 1 and not return_single_in_list:
            return positions[0]
        
        return positions