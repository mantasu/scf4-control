import time
import rospy
import serial

from queue import Queue

class SerialHandler():
    def __init__(self, verbose=False):
        self.verbose = verbose

        self.status = ""
        self.version = ""
        self.voltage = ""
        self.feedback = []

        self.port = None
        self.commands = Queue()
        self.action_connect = Queue()
        self.action_disconnect = Queue()
        self.action_recipe = Queue()

        self.config = {
            "A": {
                "jog_steps": 50,
                "steps_min": 1,
                "steps_max": 65535,
                "speed_min": 400,
                "speed_max": 4000,
                "vel_factor": 1000,
            },
            "B": {
                "jog_steps": 25,
                "steps_min": 1,
                "steps_max": 65535,
                "speed_min": 400,
                "speed_max": 4000,
                "vel_factor": 1000,
            }
        }

        self.serial = None
    
    def __verify_speed(self, speed, motor_type):
        """Verifies the motor speed

        Clamps the speed within a required interval which can afterwards
        be included in the G-code command.

        Args:
            speed (int|float|str): The speed to verify
            motor_type (str): The type of motor the speed depends on

        Returns:
            str: A suitable speed value for motors
        """
        # Convert speed to an integer value (parse string if needed)
        speed = int(speed) if isinstance(speed, str) else int(speed)

        # Ensure the speed is within a correct range
        speed = min(speed, self.config[motor_type]["speed_max"])
        speed = max(speed, self.config[motor_type]["speed_min"])

        return motor_type + str(speed)


    def __verify_steps(self, steps, motor_type):
        """Verifies the motor steps

        Clamps the steps within a required interval which can afterwards
        be included in the G-code command. Usual range is 16 bit.

        Args:
            steps (int|float|str): The steps to verify
            motor_type (str): The type of motor the steps depend on

        Returns:
            str: A suitable steps value for motors
        """
        # Convert steps to an integer value (parse string if needed)
        steps = int(steps) if isinstance(steps, str) else int(steps)

        # Get the sign and clamp steps
        sign = 1 if steps >= 0 else -1
        steps = min(abs(steps), self.config[motor_type]["steps_max"]) * sign
        steps = max(abs(steps), self.config[motor_type]["steps_min"]) * sign

        return motor_type + str(steps)


    def __generate_motor_cmd(self, callback, args, cmd=""):
        for i, val in enumerate(args):
            if val is not None:
                motor_type = chr(ord('@') + i + 1)
                motor_speed = callback(val, motor_type)
                cmd += " " + motor_speed
        
        return cmd

    
    def __serial_send(self, command):
        """Sends a motor control command through serial port.

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
        try:
            # Send the G-code command and get the response
            self.serial.write(bytes(command + "\r\n", "utf8"))
            response = self.serial.readline().decode("utf-8").strip()
        except AttributeError:
            # If the connection hasn't been established
            rospy.logerr("Connect to the port first!")

        return response
    
    def _init_controller(self):
        """Initializes the motor controller and its driver.

        The initialization process (G-code commands):
            1. Reset and initialize motor driver
            2. Set stepping (should be 64 by default)
            3. Set normal move
            4. Switch to relative coordinate programming mode
            5. Energize PI LED
            6. Set motor and DN drive current
            7. Set motor idle current
            8. Set PI low/high detection voltage
        """
        # Initialize the motor controller via G-code commands
        self.__serial_send("$B2")
        self.__serial_send("M243 C6")
        self.__serial_send("M230")
        self.__serial_send("G91")
        self.__serial_send("M238")
        self.__serial_send("M234 A190 B190 C190 D90")
        self.__serial_send("M235 A120 B120 C120")
        self.__serial_send("M232 A400 B400 C400 E700 F700 G700")

        # Inform the SCF4 controller has been initialized
        rospy.loginfo("The controller has been initialized")
    
    def get_version(self):
        """Returns the version information.

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
        return self.__serial_send("$S")
    
    def get_status(self):
        """Returns motor position, limit switch & moving status.t

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
        return self.__serial_send("!1")    

    def get_voltage(self):
        """Returns the ADC voltage

        Sends G-code via serial port to retrieve the voltage in the
        format of "ADC={val}", where {val} is some 12 bit decimal value.

        Returns:
            str: Information about voltage
        """
        return self.__serial_send("M231")


    def connect(self, port, baudrate, timeout):
        
        rospy.loginfo(f"Connecting via port {port}...")

        self.serial = serial.Serial()
        self.serial.port = port
        self.serial.baudrate = baudrate
        self.serial.timeout = timeout

        self.serial.open()
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

        rospy.loginfo("Connected")
    
    def disconnect(self):
        try:
            # Close connection
            self.serial.close()
            self.serial = None
            
            # Inform connection closed
            rospy.loginfo("Disconnected")

        except AttributeError:
            # If the connection isn't established
            rospy.loginfo("Already disconnected")
    
    

    def send(self, data):
        self.commands.put(data)

    def serial_worker(self):
        idle_counter = 0
        status_version = ""
    
    

    def change_speed(self, *args):
        cmd = self.__generate_motor_cmd(self.__verify_speed, args, "M240")
        self.__serial_send(cmd)

    def move_motors(self, *args):
        cmd = self.__generate_motor_cmd(self.__verify_steps, args, "G0")
        self.__serial_send(cmd)