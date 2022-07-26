import rospy
from scf4_control.serial.serial_base import SerialBase
from scf4_control.utils import adc_to_volt

class SerialHandler(SerialBase):
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
        # Log the controller initialization started
        rospy.loginfo("Initializing controller...")

        # Initialize the motor controller via G-code commands
        self.send_command("$B2")
        self.send_command("M243 C6")
        self.send_command("M230")
        self.send_command("G91")
        self.send_command("M238")
        self.send_command("M234 A190 B190 C190 D90")
        self.send_command("M235 A120 B120 C120")
        self.send_command("M232 A400 B400 C400 E700 F700 G700")

        # Retrieve the minimum speed for each motor and set it
        self.set_speed(*self.retrieve_motors_prop("speed_min"))
    
    def _init_motors(self):
        # Log that motor initialization started
        rospy.loginfo("Initializing motors...")

        # Determine where to move based on switch
        status = self.get_status().split(", ")
        motors = "ABC"[:self.N_MOTORS]
        steps = [
            100 * (-1 if status[3] == "1" else 1),
            100 * (-1 if status[4] == "1" else 1),
            100 * (-1 if status[5] == "1" else 1),
        ][:self.N_MOTORS]

        # Move moors towards switch
        self.set_coordinate_mode(1)
        self.set_motor_move_mode(1)
        self.move(*steps)
        self.await_idle(*motors, initial_status=status)

        # Go a bit back from switch
        self.set_motor_move_mode(0)
        self.move(*[-200]*self.N_MOTORS)
        self.await_idle()

        # Move till switch reached
        self.set_motor_move_mode(1)
        status = self.get_status()
        self.move(*[100]*self.N_MOTORS)
        self.await_idle(*motors, initial_status=status)

        # Set the current coordinate where the switch is as middle
        self.set_counter(*self.retrieve_motors_prop("switch_pos"))
        self.set_motor_move_mode(0)
        self.set_coordinate_mode(0)

        # Move zoom all the way back to see a wide view and adjust focus
        self.move(self.config['A']["count_def"], self.config['B']["count_def"])
        self.await_idle()
        self.set_coordinate_mode(1)
    
    def retrieve_motors_prop(self, prop):
        """Retrieves a config property for every motor

        Takes a name of a config property that each motor has and gets
        that property for every motor.

        Args:
            prop (str): The name of the motor config attribute

        Returns:
            lst: A list of properties corresponding to motors
        """

        return [self.config[chr(i+65)][prop] for i in range(self.N_MOTORS)]
    
    def is_equal(self, *args, status_group=2, vals="1"):
        """Checks if any of the motor status is equal to some value

        Reads the motor status which comes in 3 sub-statuses containing
        3 values (one for each motor). It selects the sub-status and
        checks if the specified motor(-s) has the same value(-s) as in
        the sub-status.

        Note: the first sub-status corresponds to motor position, the
            second to whether the switch is triggered and the third -
            to whether the motor is moving towards some coordinate.

        Args:
            status_group (int, optional): The index of the sub-status - 
                one of `0`, `1`, or `2`. Defaults to 2.
            vals (str|list, optional): The values to check the motor
                status against. Defaults to "0".

        Returns:
            bool: Whether the motor status is equal to the given value
        """
        # Init the status
        is_equal = True
        start, end = [(0, 3),(3, 6),(6, 9)][status_group]
        status = self.get_status().split(", ")[start:end]

        if len(status) != 3:
            raise RuntimeError("Please don't spam the command too often")

        if not isinstance(vals, list):
            # If single value
            vals = [vals] * 3
        else:
            # Ensure each value's stringified
            vals = [str(val) for val in vals]

        if len(args) == 0:
            # Check all motors
            return vals == status

        for motor_type, val in zip(args, vals):
            # Update with specific motor type
            status_idx = ord(motor_type) - 65
            is_equal = is_equal and (status[status_idx] == val)
        
        return is_equal

    def is_at(self, *args, **kwargs):
        """Checks if the specified motor(-s) is at specified position

        Gets the status for the motors and checks the first 3 values -
        one for every motor. Each value represents motor's position.
        `False` is returned only if either of the specified motor(-s)
        or, if not specified, either of all the motors, does not match
        the specified position.

        Args:
            *args: The motor positions to check. If either of the value
                is None, that motor is skipped
            **kwargs: The motor name-value pairs. This is an alternative
                to args, in case the motor value is specified by its
                keyword name, e.g., b=0 

        Returns:
            bool: Whether the specified motor(-s) is at given position
        """
        # Convert the provided args and kwargs to motor name-value pairs
        motor_types = self._to_motor_types(args, kwargs, convert="count")
        args, vals = motor_types.keys(), list(motor_types.values())

        return self.is_equal(*args, status_group=0, vals=vals)

    def is_moving(self, *args):
        """Checks if any of the motors are moving towards some position

        Gets the status for the motors and checks the last 3 values -
        one for every motor. `0` indicates not moving and `1` indicates
        a motor is moving. `False` is returned only if the specified
        motor(-s) or, if not specified, all of the motors, are not
        moving.

        Args:
            *args: The type of motors to check if they're moving. For
                example 'A', 'C'. If nothing provided, all motors are
                checked

        Returns:
            bool: Whether the motors are currently moving
        """
        return self.is_equal(*args, status_group=2, vals="1")
    
    def is_switched(self, *args, vals="1"):
        """Checks if any of the motors triggered switch state

        Gets the status for the motors and checks the centre 3 values -
        one for every motor. `0` indicates not switched and `1`
        indicates a switch for that motor is triggered. `False` is
        returned only if the specified motor(-s) or, if not specified,
        all of the motors, did not trigger switch.

        Args:
            *args: The type of motors to check if their switch status is
                triggered. For example 'A', 'C'. If nothing provided,
                all motors are checked
            vals (str|list, optional): The values to check each motor
                against. `0` - not switched, `1` - switched

        Returns:
            bool: Whether the motors are currently moving
        """
        return self.is_equal(*args, status_group=1, vals=vals)

    def await_idle(self, *args, **kwargs):
        """Halts till motors stop moving

        Enters an infinite `while` loop until the controller returns the
        motor status as "matches the target status" (for the specified
        motor(-s) or, if not specified, for every motor). If no initial
        status is provided, it assumes the target status is `0` for
        status group `2`, i.e., not going towards some goal. Otherwise,
        if initial_status is provided, it assumes the target status is
        the opposite of the given status for every specified motor (or
        for all, if motors are not specified) for the status group `1`,
        i.e., the switch is (not) triggered.

        Args:
            *args: The type of motors to check if they're moving. For
                example 'A', 'C'. If nothing provided, all motors are
                checked
            **kwargs: The additional keyword arguments specifying how to
                wait for the idle status. The parameters are as follows:
                    * initial_status (str|list|int, optional): The
                        initial status of the motor(-s) of interest. Can
                        be the whole status message, the same message
                        but chopped to substrings, the specific values
                        (str|int) for the motor(-s) identifying their
                        status (either standalone or in list). If a
                        single value is given but more motors are
                        provided, same value is applied to all of them.
                        Defaults to None.
                    * status_group (int, optional): The index of the
                        sub-status - one of `0`, `1`, or `2`. Defaults
                        to 2.
                    * timeout (int, optional): The time to wait (in
                        seconds) before forcefully stopping to wait.
                        Defaults to 10.
                    * on_timeout (str, optional): The action to do if a
                        timeout occurs - raise error or just log. One of
                        "raise"|"log". Defaults to "raise".
                    * sleep_time (float, optional): The time (in
                        seconds) to wait before each call of the status
                        that is used to check if the goal is reached.
                        Defaults to 0.1.

        Raises:
            RuntimeError: If the timeout has been reached
        """
        # Get keyword arguments or take default vals
        initial_status = kwargs.pop("initial_status", None)
        default_status = 2 if initial_status is None else 1
        status_group = kwargs.pop("status_group", default_status)
        timeout = kwargs.pop("timeout", 10)
        on_timeout = kwargs.pop("on_timeout", "raise")
        sleep_time = kwargs.pop("sleep_time", 0.1)

        if initial_status is not None:
            # A function that maps initial val to target val
            to_val = lambda x: '1' if str(x) == '0' else '0'

            if isinstance(initial_status, str) and ", " in initial_status:
                # Convert to array of strings if full status
                initial_status = initial_status.split(", ")
            
            if isinstance(initial_status, list):
                if len(initial_status) > 3:
                    # Extract the specific sub-status which interests us
                    start, end = [(0, 3), (3, 6), (6, 9)][status_group]
                    initial_status = initial_status[start:end]
                
                # Apply the mapping function to get vals
                vals = list(map(to_val, initial_status))
            else:
                # If just a single value given
                vals = to_val(initial_status)
        else:
            # Otherwise
            vals = "0"

        # Check the rospy start time
        start_time = rospy.get_time()

        while True:
            if self.is_equal(*args, status_group=status_group, vals=vals):
                # If the target condition is met, terminate the waiting
                break
            
            if rospy.get_time() - start_time > timeout:
                if on_timeout == "raise":
                    # If the runtime error should be raised on timeout
                    raise RuntimeError("Waited too long for motors to stop")
                elif on_timeout == "log":
                    # If the timeout  error ony needs to be logged out
                    rospy.logerr("Waited too long for motors to stop")
                    break
            
            # Sleep for given time
            rospy.sleep(sleep_time)

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
    
    def sweep_once(self, motor, forth=True, reach_start=True, callback=None):
        # Get minimum and maximum position counts
        pos_min = self.config[motor]["count_min"]
        pos_max = self.config[motor]["count_max"]

        # Coordinate mode absolute
        self.set_coordinate_mode(0)

        if not forth:
            # Just swap the min and max values
            pos_min, pos_max = pos_max, pos_min

        if reach_start:
            # If the start of the position must be reached before sweep
            move_cmd = [pos_min if m == motor else None for m in "ABC"]
            self.move(*move_cmd)
            self.await_idle(motor)
        
        # Generate the moving command for the specified motor type
        move_cmd = [pos_max if m == motor else None for m in "ABC"]
        self.move(*move_cmd)

        while True:
            if not self.is_moving(motor):
                # If moving has stopped
                break
            
            if callback is not None:
                # If something needs to be done during 
                callback(self.get_motor_position(motor))
        
        # Coordinate mode relative
        self.set_coordinate_mode(1)
            
