"""Methods for interacting with an mbed over a serial connection.

This file is part of the code for the Hills Road/Systemetric entry to
the 2017 Student Robotics competition "Easy as ABC".
"""


import serial
import time


class CommandFailureError(Exception):
    """The mbed responded with an error."""


class MovementInterruptedError(CommandFailureError):
    """The robot's movement was interrupted."""


class Mbed(object):
    def __init__(self, log, timeout=None):
        self.log = log
        port = "/dev/ttyACM0"
        baudrate = 115200
        self.conn = serial.Serial(port, baudrate=baudrate, timeout=timeout, writeTimeout=timeout)

    def get_switch_state(self):
        try:
            self.conn.write("s")
        except serial.SerialTimeoutException:
            self.log.error("Timeout sending mbed command s")
            return
        while not self.conn.inWaiting():
            pass
        response = ord(self.conn.read(1))
        self.log.debug("mbed sent response %s", response)
        self.conn.flushInput()
        return response

    def move(self, amount, ignore_crash=False):
        # type: (...) -> None
        """
        Move the motors `abs(amount)`cm.
        If negative, go backwards
        """
        try:
            if amount > 0:
                self.forwards(amount)
            elif amount == 0:
                self.log.debug("Told to move by nothing. No command will be sent.")
            else:
                self.backwards(abs(amount))
        except MovementInterruptedError:
            if not ignore_crash:
                raise

    def turn(self, amount):
        # type: (...) -> None
        """
        Turn amount degrees right (clockwise)
        This function is clever and will turn the right direction
        """
        self.log.debug("Told to turn %s degrees", amount)
        amount %= 360
        self.log.debug("turn mod 360 is %s", amount)
        if amount > 180:
            self.turn_left(360 - amount)
        elif amount == 0:
            self.log.debug("Told to turn by nothing. No command will be sent.")
            return
        else:
            self.turn_right(amount)

    def forwards(self, amount):
        # type: (float) -> None
        """
        Go forwards `amount` m.
        """
        if amount <= 2.55:
            try:
                self.send_command("f", int(amount * 100))
            except CommandFailureError as e:
                self.log.exception("Failed to move forwards (short movement)")
                raise MovementInterruptedError
        else:
            amount, remainder = divmod(amount * 100, 10)  # decimetres, centimetres
            try:
                self.send_command("F", int(amount))
            except CommandFailureError as e:
                self.log.exception("Failed to move forwards (long movement phase 1)")
                raise MovementInterruptedError
            else:
                if remainder >= 2:
                    try:
                        self.send_command("f", int(remainder))
                    except CommandFailureError as e:
                        self.log.exception("Failed to move forwards (long movement phase 2)")
                        raise MovementInterruptedError
                else:
                    self.log.warn("Discarding extra distance of %s cm", remainder)

    def backwards(self, amount):
        # type: (float) -> None
        """
        Go backwards `amount` m.
        """
        try:
            self.send_command("b", int(amount*100))
        except CommandFailureError as e:
            self.log.exception("Failed to move backwards")
            raise MovementInterruptedError()

    def turn_left(self, amount):
        # type: (float) -> None
        """
        Turn left `amount` degrees.
        This function is not clever and will turn more than 180 degrees if asked.
        """
        self.log.debug("Turning left %s degrees", amount)
        try:
            self.send_command("l", int(round(amount)))
        except CommandFailureError as e:
            self.log.exception("Failed to turn left")
            raise MovementInterruptedError()

    def turn_right(self, amount):
        # type: (float) -> None
        """
        Turn right `amount` degrees.
        This function is not clever and will turn more than 180 degrees if asked.
        """
        self.log.debug("Turning right %s degrees", amount)
        try:
            self.send_command("r", int(round(amount)))
        except CommandFailureError as e:
            self.log.exception("Failed to turn right")
            raise MovementInterruptedError()
            
    def retry(self):
        """
        Retry the previous command on the mbed
        """
        self.log.debug("Continuing")
        try:
            self.send_command("c")
        except CommandFailureError:
            self.log.exception("Failed to continue")
            raise MovementInterruptedError()

    def send_command(self, command, data=None):
        # type: (str, int) -> None
        """
        Send a command (character) to the mbed with 1 byte of data.

        Raises:
            CommandFailureError: The mbed responded with an error.
        """
        self.log.debug("Starting mbed command %s(%s)", command, data)
        send_time = time.time()
        try:
            self.conn.write(command)
            if data is not None:
                self.conn.write(chr(data))
        except serial.SerialTimeoutException:
            self.log.exception("Timeout sending mbed command %s(%s)!", command, data)
            return
        while not self.conn.inWaiting():
            pass
        response = self.conn.read(1)
        rtt = int(round(time.time() - send_time, 2))
        self.log.debug("mbed sent response %s after %s seconds", response, rtt)
        self.conn.flushInput()
        if response == "e":
            self.log.warn("Command failed!")
            raise CommandFailureError(response)
        else:
            self.log.debug("Completed command %s(%s) -> %s after %s seconds", command, data, response, rtt)
