import rospy
import serial
import threading

from scf4_control.utils import clamp

class SerialBase():
    def __init__(self, serial_config, motors_config):
        # Merge both configs and initialize serial object
        self.config = {**serial_config, **motors_config}
        self.N_MOTORS = sum([key in "ABC" for key in motors_config.keys()])
        self.serial = None
        self.coordinate_mode = 1
        self.motor_move_mode = 0
        self.read_lock = threading.Lock()
    
    def _verify_steps(self, steps, motor_type):
        """Verifies the motor steps

        Clamps the steps within a required interval which can afterwards
        be included in the G-code command. Usual range is 16 bit.

        Args:
            steps (int|float|str): The steps to verify
            motor_type (str): The type of motor the steps depend on

        Returns:
            str: A suitable steps value for motors
        """
        range_type = "steps" if self.coordinate_mode else "count"

        if isinstance(steps, str) and steps in ["min", "max", "jog"]:
            steps = self.config[motor_type][f"{range_type}_{steps}"]
            return motor_type + str(steps)
        
        # Float/str to int
        steps = int(steps)

        # Get the type of range for min and max values and clamp
        min_val = self.config[motor_type][f"{range_type}_min"]
        max_val = self.config[motor_type][f"{range_type}_max"]
        clamped = clamp(steps, min_val, max_val, self.coordinate_mode)

        return motor_type + str(clamped)
    
    def _verify_speed(self, speed, motor_type):
        """Verifies the motor speed

        Clamps the speed within a required interval which can afterwards
        be included in the G-code command.

        Args:
            speed (int|float|str): The speed to verify
            motor_type (str): The type of motor the speed depends on

        Returns:
            str: A suitable speed value for motors
        """
        if isinstance(speed, str) and speed in ["min", "max", "def"]:
            speed = self.config[motor_type][f"speed_{speed}"]
            return motor_type + str(speed)

        # Float/str to int
        speed = int(speed)

        if speed < 0:
            # Nothing
            return ""

        # Ensure the speed is within a correct range
        speed = min(speed, self.config[motor_type]["speed_max"])
        speed = max(speed, self.config[motor_type]["speed_min"])

        return motor_type + str(speed)

    def _verify_count(self, count, motor_type):
        """Verifies the motor position counter

        Clamps the count within a required interval which can afterwards
        be included in the G-code command.

        Args:
            count (int|float|str): The position counter to verify
            motor_type (str): The type of motor the speed depends on

        Returns:
            str: A suitable counter value for motors
        """
        # Float/str to int
        count = int(count)

        if count < 0:
            # Nothing
            return ""

        # Ensure the count is within a correct range
        count = min(count, self.config[motor_type]["count_max"])
        count = max(count, self.config[motor_type]["count_min"])

        return motor_type + str(count)

    def _generate_motor_cmd(self, callback, args, cmd=""):
        """Generates a motor control command given motor values.

        Given subsequent values for every motor, it generates a command
        represented in the form of "A{val1} B{val2}" where A, B - Motor
        names, {val1}, {val2} - provided values. There will be as many
        motor names in alphabetical order as there are values. Some
        motor names may be skipped if the value is `None`.

        Args:
            callback (function): The function which combines the motor
                type/name and its value
            args (list): The list of values corresponding to each motor
                in sequence. A value can be `None` if the motor should
                be skipped
            cmd (str, optional): The start of the command to which the
                generated motor-value correspondences should be appended
                to. Defaults to "".

        Returns:
            str: A full G-code command (or a part of it if `cmd` is not
                provided) including all the motor-value correspondences.
        """
        for i, val in enumerate(args):
            if val is not None:
                # Generate motor name and update cmd
                motor_type = chr(ord('@') + i + 1)
                motor_val = callback(val, motor_type)
                cmd += " " + motor_val
        
        return cmd

    def connect(self, port=None, baudrate=None, timeout=None):
        """Initializes connection via serial port to SCF4

        This method connects to SCF4 module by initializing a serial
        object which can send G-code commands to the control module.

        Note: if either of the arguments is not provided, it is replaced
            by a default one which is a class attribute.

        Args:
            port (str, optional): The name of the USB port SCF4 is
                connected to. Defaults to None.
            baudrate (int, optional): The rate at which information is
                transferred. Defaults to None.
            timeout (int, optional): The time (ms) to wait before
                terminating the waiting process for the response after
                sending a command through serial. Defaults to None.
        """
        # Choose custom or default port, baudrate and timeout
        port = self.config["port"] if port is None else port
        timeout = self.config["timeout"] if timeout is None else timeout
        baudrate = self.config["baudrate"] if baudrate is None else baudrate
        
        # Inform which port the connection is occurring
        rospy.loginfo(f"Connecting via port {port}...")

        # Initialize the serial object by specifying its parameters
        self.serial = serial.Serial(port, baudrate, timeout=timeout)

        # Reset input and output buffers
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
    
    def disconnect(self):
        """Closes communication via serial port

        If the serial connection exists, it is closed and the serial
        object is deleted.
        """
        try:
            # Close connection
            self.serial.close()
            self.serial = None
            
            # Inform connection closed
            rospy.loginfo("Disconnected")

        except AttributeError:
            # If the connection isn't established
            rospy.loginfo("Already disconnected")
    
    def send_command(self, command):
        """Sends a motor control command through serial port

        Converts a G-code command to bytes (UTF-8) nad sends the data
        to the connected SCF4 control module through the serial port.
        After the command is executed on the hardware, the status is
        returned.

        Note: as stated in `PySerial documentation`__: "If the \n is
            missing in the return value, it returned on timeout."

        TODO: define this link: https://pyserial.readthedocs.io/en/latest/shortintro.html#readline for documentation according to 
        https://stackoverflow.com/questions/14414878/how-do-i-break-a-link-in-a-rst-docstring-to-satisfy-pep8

        Args:
            command (str): The command in G-code style to parse

        Returns:
            str: A response from the control module
        """
        #print(f">> {command}")
        try:
            # Acquire the lock first
            self.read_lock.acquire()

            # Send the G-code command and get the response
            self.serial.write(bytes(command + "\r\n", "utf8"))
            response = self.serial.readline().decode("utf-8").strip()
        except AttributeError:
            # If the connection hasn't been established
            rospy.logerr("Connect to the port first!")
        finally:
            # Assure always released
            self.read_lock.release()
        #print(f"<< {response}")
        return response
    
    def get_version(self):
        """Returns the version information

        Sends G-code via serial port to retrieve version information
        about firmware, PCB revision, manufacturer & serial number.

        For example, the following output could be decomposed as follows
        `EVB.1.0.2, SCF4-M RevB, Kurokesu, 5DDFF39-3739584E-xxxxxxxx`: 
            * `EVB.1.0.2` - Module firmware version
            * `SCF4-M RevB` - Module PCB revision
            * `Kurokesu` - Manufacturer Brand
            * `5DDFF39-3739584E-xxxxxxxx` - Unique serial number

        Returns:
            str: Information about version
        """
        return self.send_command("$S")
    
    def get_status(self):
        """Returns motor position, limit switch & moving status

        Sends G-code via serial port to retrieve position,
        limit switch status, and moving status for every motor.

        For example, the following output could be decomposed as follows
        `4000, 20000, 0, 0, 0, 0, 0, 0, 0`:
            * `4000 20000 0` - Position Status for motors A, B and C
            * `0 0 0` - Switch Status for motors A, B and C
            * `0 0 0` - Moving Status for motors A, B and C

        Returns:
            str: Information about motor status
        """
        return self.send_command("!1")    

    def get_voltage(self):
        """Returns the ADC voltage

        Sends G-code via serial port to retrieve the voltage in the
        format of "ADC={val}", where {val} is some 12 bit decimal value.

        Returns:
            str: Information about voltage
        """
        return self.send_command("M247")

    def move(self, *args):
        """Instructs the motors to move by a certain amount of steps

        Takes a list of step values, assigns them to each motor and
        generates a G-code command which is sent via serial port to
        instruct the motors to move. The values are clamped to be within
        a correct range.

        Args:
            *args: Variable length list of step values. Each value
                corresponds to a single motor in that sequence. A value
                can be of type str|int|float or `None` if the motor
                should be ignored.
        """
        # TODO: kwargs for a/b/c, maybe single verify function

        # Generate G-code cmd by assigning values to motors and send it
        cmd = self._generate_motor_cmd(self._verify_steps, args, "G0")
        self.send_command(cmd)

    def set_speed(self, *args):
        """Instructs to change the motors speed

        Takes a list of speed values, assigns them to each motor and
        generates a G-code command which is sent via serial port to
        instruct the motors to change their speed. The values are
        clamped to be within a correct range.

        Args:
            *args: Variable length list of speed values. Each value
                corresponds to a single motor in that sequence. A value
                can be of type str|int|float or `None` if the motor
                should be ignored.
        """
        # Generate G-code cmd by assigning values to motors and send it
        cmd = self._generate_motor_cmd(self._verify_speed, args, "M240")
        self.send_command(cmd)
    
    def set_counter(self, *args):
        """Instructs to set a custom position counter for the motors

        Takes a list of counter values, assigns them to each motor and
        generates a G-code command which is sent via serial port to
        instruct the motors to change their position counter value. The
        values are clamped to be within a correct range.

        Args:
            *args: Variable length list of counter values. Each value
                corresponds to a single motor in that sequence. A value
                can be of type str|int|float or `None` if the motor
                should be ignored.
        """
        # Generate G-code cmd by assigning values to motors and send it
        cmd = self._generate_motor_cmd(self._verify_count, args, "G92")
        self.send_command(cmd)

    def set_filter_position(self, type):
        """Sets the position of the IR filter

        Sends a G-code command via serial port to tell the SCF4
        controller to change the filter position to one of the 2
        locations.

        Args:
            type (int): The type of position: `0` (first location) or
                `1` (second location)
        """
        # Generate G-code cmd and send it
        cmd = "M7" if type == 0 else "M8"
        self.send_command(cmd)

    def set_coordinate_mode(self, type):
        """Sets the motor coordinate mode to "absolute" or "relative"

        Sends a G-code command via serial port to tell the SCF4
        controller to change the coordinate mode to either absolute or
        relative. More precisely:
            * _Absolute_ - the range is limited by the internal 16-bit
                motor register
            * _Relative_ - motors can be instructed to turn positive or
                negative with a max step size of 16 bits. Anything
                beyond will overflow but won't affect the motors.

        Args:
            type (int): The type of coordinate mode: `0` (absolute) or
                `1` (relative)
        """
        # Mode to absolute/relative
        self.coordinate_mode = type

        # Generate G-code cmd and send it
        cmd = "G90" if type == 0 else "G91"
        self.send_command(cmd)
    
    def set_motor_move_mode(self, type, *args):
        """Sets the motor movement mode to "normal" or "forced"

        Sends a G-code command via serial port to tell the SCF4
        controller to change the movement mode to either normal or
        forced. More precisely:
            * _Normal_ - the motors turn the instructed step count and
                stop after
            * _Forced_ - the motors don't stop after the step count is
                reached (internal counter may overflow). Motion is
                stopped when input status changes, reset command is
                issued or "M0" (:func:`~SerialHandler.stop`)

        Args:
            type (int): The type of movement mode: `0` (normal) or
                `1` (forced)
            *args: Additional values of type str specifying motors to
                target, e.g., "A". If none, all are targeted
        """
        # Set mode to normal/forced
        self.motor_move_mode = type
        
        if len(args) == 0:
            # By default
            args = "ABC"

        # Generate G-code command and send it
        cmd = "M230" if type == 0 else "M231"
        cmd += " " + " ".join(args)
        self.send_command(cmd)
    
    def wait(self, time):
        """Instructs the controller stall by some time (miliseconds)

        Given a delay time [ms], a command is sent to the SCF4
        controller to stall by that amount of time. During this time,
        command parser is blocked and the command is only completed
        after the time passes.

        Args:
            time (int): The time (number of miliseconds) to wait
        """
        cmd = "G4 " + str(abs(int(time)))
        self.send_command(cmd)

    def stop(self):
        """Forcefully stops the motor movement

        A command is sent to the controller which instructs the motors
        to stop moving if it is not in a (:func:`~SerialHandler.wait`)
        mode.
        """
        self.send_command("M0")