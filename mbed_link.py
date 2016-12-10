
import serial


class StepperMotors():
    def __init__(self, port, timeout=0.2):
        self.mbed = serial.Serial(port, timeout=timeout, writeTimeout=timeout)

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
        amount %= 360
        if amount > 180:
            self.turn_left(360-amount)
        else:
            self.turn_right(amount)
        
    
    def forwards(self, amount):
        """
        Go forwards `amount` cm.
        """
        
    def backwards(self, amount):
        """
        Go backwards `amount` cm.
        """
    
    def turn_left(self, amount):
        """
        Turn left `amount` degrees.
        This function is not clever and will turn more than 180 degrees if asked.
        """
    
    def turn_right(self, amount):
        """
        Turn right `amount` degrees.
        This function is not clever and will turn more than 180 degrees if asked.
        """