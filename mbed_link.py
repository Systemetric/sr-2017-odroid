
import serial


class StepperMotors():
    def __init__(self, port, timeout=0.2):
        self.mbed = serial.Serial(port, timeout=timeout, writeTimeout=timeout)

    def move(self, amount: int):
        """
        Move the motors `abs(amount)`cm.
        If negative, go backwards
        """
        if amount > 0:
            self.forwards(amount)
        else:
            self.backwards(amount)
    
    def turn(self, amount: int):
        """
        Turn amount degrees right (clockwise)
        This function is clever and will turn the right direction
        """
        amount %= 360
        if amount > 180:
            self.turn_left(360-amount)
        else:
            self.turn_right(amount)
        
    
    def forwards(self, amount: int):
        """
        Go forwards `amount` cm.
        """
        
    def backwards(self, amount: int):
        """
        Go backwards `amount` cm.
        """
    
    def turn_left(self, amount: int):
        """
        Turn left `amount` degrees.
        This function is not clever and will turn more than 180 degrees if asked.
        """
    
    def turn_right(self, amount: int):
        """
        Turn right `amount` degrees.
        This function is not clever and will turn more than 180 degrees if asked.
        """