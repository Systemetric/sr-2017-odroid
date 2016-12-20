import serial


class StepperMotors():
    def __init__(self, log, timeout=None):
        self.log = log
        port = '/dev/ttyACM0'
        baudrate = 115200
        self.mbed = serial.Serial(port, baudrate=baudrate, timeout=timeout, writeTimeout=timeout)
        self.lastTurn = ''

    def move(self, amount):
        """
        Move the motors `abs(amount)`cm.
        If negative, go backwards
        """
        if amount > 0:
            self.forwards(amount)
        else:
            self.backwards(amount)

    def turn(self, amount):
        """
        Turn amount degrees right (clockwise)
        This function is clever and will turn the right direction
        """
        self.log.debug("Told to turn %s degrees", amount)
        amount %= 360
        self.log.debug("turn mod 360 is %s", amount)
        if amount > 180:
            self.turn_left(360 - amount)
        else:
            self.turn_right(amount)

    def forwards(self, amount):
        """
        Go forwards `amount` m.
        """
        self.send_command("f", int(amount*100))

    def backwards(self, amount):
        """
        Go backwards `amount` m.
        """
        self.send_command("b", int(amount*100))

    def turn_left(self, amount):
        """
        Turn left `amount` degrees.
        This function is not clever and will turn more than 180 degrees if asked.
        """
        self.log.debug("Turning left %s degrees", amount)
        self.send_command("l", int(amount))
        self.lastTurn = "Left"

    def turn_right(self, amount):
        """
        Turn right `amount` degrees.
        This function is not clever and will turn more than 180 degrees if asked.
        """
        self.log.debug("Turning right %s degrees", amount)
        self.send_command("r", int(amount))
        self.lastTurn = "Right"

    def send_command(self, command, data):
        """
        Send the `command` (character) to the mbed with 1 byte of data (int)
        """
        self.log.debug("Starting mbed command {}({})".format(command, data))
        try:
            self.mbed.write(command)
            self.mbed.write(chr(data))
        except serial.SerialTimeoutException:
            self.log.error("Timeout sending {} character to mbed with data {}. Not retrying.".format(command, data))
            return "Error"
        while not self.mbed.inWaiting():
            pass
        response = self.mbed.read(1)
        if response != "d":
            self.log.error("Mbed sent us a bad response. May or may not have done `{}`".format(command))
            return "Error"
        self.mbed.flushInput()
        self.log.debug("Sucessfully completed {}({})".format(command, data))
