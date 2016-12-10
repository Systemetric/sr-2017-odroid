
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
        
    def forwards(self, amount: int):
        pass
        
    def backwards(self, amount: int):
        pass