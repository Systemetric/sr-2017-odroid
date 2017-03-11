import serial
import time


class IOBoard(object):
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

    def move(self, amount):
        """
        Move the motors `abs(amount)`cm.
        If negative, go backwards
        """
        if amount > 0:
            self.forwards(amount)
        else:
            self.backwards(abs(amount))

    def turn(self, amount):
        """
        Turn amount degrees right (clockwise)
        This function is clever and will turn the right direction
        """
        self.log.debug("Told to turn %s degrees", amount)
        amount %= 360
        self.log.debug("turn mod 360 is %s", amount)
        if amount > 180:
            movement = self.turn_left(360 - amount)
        else:
            movement = self.turn_right(amount)
        if movement == 'Error':
            self.log.error("Failed to turn")
            return 'Error'

    def forwards(self, amount):
        """
        Go forwards `amount` m.
        """
        if amount <= 2.55:
            self.send_command("f", int(amount * 100))
        else:
            amount, remainder = divmod(amount * 100, 10)
            movement = self.send_command("F", int(amount))
            if movement == 'Error':
                self.log.error("Failed to move forwards")
                return 'Error'
            if remainder >= 2:
                self.send_command("f", int(remainder))
            else:
                self.log.warn("Discarding extra distance of %s cm", remainder)

    def backwards(self, amount):
        """
        Go backwards `amount` m.
        """
        movement = self.send_command("b", int(amount*100))
        if movement == 'Error':
            self.log.error("Failed to move backwards")
            return 'Error'

    def turn_left(self, amount):
        """
        Turn left `amount` degrees.
        This function is not clever and will turn more than 180 degrees if asked.
        """
        self.log.debug("Turning left %s degrees", amount)
        movement = self.send_command("l", int(round(amount)))
        if movement == 'Error':
            self.log.error("Failed to turn left")
            return 'Error'

    def turn_right(self, amount):
        """
        Turn right `amount` degrees.
        This function is not clever and will turn more than 180 degrees if asked.
        """
        self.log.debug("Turning right %s degrees", amount)
        movement = self.send_command("r", int(round(amount)))
        if movement == 'Error':
            self.log.error("Failed to turn right")
            return 'Error'

    def send_command(self, command, data):
        """
        Send the `command` (character) to the mbed with 1 byte of data (int)
        """
        self.log.debug("Starting mbed command %s(%s)", command, data)
        send_time = time.time()
        try:
            self.conn.write(command)
            self.conn.write(chr(data))
        except serial.SerialTimeoutException:
            self.log.error("Timeout sending mbed command %s(%s). Not retrying.", command, data)
            return "Error"
        while not self.conn.inWaiting():
            pass
        response = self.conn.read(1)
        self.log.debug("mbed sent response %s", response)
        self.conn.flushInput()
        if response == 'e':
            self.log.error("Failed to complete movement")
            return "Error"
        self.log.debug("Sucessfully completed command %s(%s) after %s seconds", command, data, time.time() - send_time)
