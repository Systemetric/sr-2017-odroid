import time


class DCMotors():
    def __init__(self, log, motors):
        self.log = log
        self.motors = motors
        self.lastTurn = ''

    def forwards(self, distance, speed=0.75, ratio=-1.05, speed_power=80):
        """
        Go forwards (distance) meters
        """
        # Sasha Shtyrov decreased the power to the motors to prevent current overload
        if distance < 0:
            self.log.warning("robot.forwards() passed a negative distance, inverting!")
            distance = -distance
        power = speed * speed_power
        sleep_time = distance / speed
        self.log.info("Moving forwards %s meters", distance)
        # adding lines to start motors at lower power then up to full power
        self.motors[0].m0.power = power * ratio * 0.7
        self.motors[1].m1.power = power * 0.7
        t_end = time.time() + 0.02
        while time.time() < t_end:
            self.log.info("starting %s Amps, starting %s Volts", self.power.battery.current, self.power.battery.voltage)
            # time.sleep(0.001)
        # close of code to start slower
        self.motors[0].m0.power = power * ratio
        self.motors[1].m1.power = power
        t_end = time.time() + sleep_time
        while time.time() < t_end:
            self.log.info("current draw is %s Amps, voltage draw is %s Volts", self.power.battery.current, self.power.battery.voltage)
            self.log.info("time left is %s milliseconds", (t_end - time.time()))
            # time.sleep(0.001) implicit wait from log code
        self.log.info("current draw is %s Amps, voltage draw is %s Volts", self.power.battery.current, self.power.battery.voltage)
        # adding lines to slow down motors at lower power then up to zero power
        self.motors[0].m0.power = power * ratio * 0.7
        self.motors[1].m1.power = power * 0.7
        t_end = time.time() + 0.02
        while time.time() < t_end:
            self.log.info("stopping %s Amps, stopping %s Volts", self.power.battery.current, self.power.battery.voltage)
            time.sleep(0.001)
        # close of code to stop slower
        self.motors[0].m0.power = 0
        self.motors[1].m1.power = 0
        t_end = time.time() + 0.04
        while time.time() < t_end:
            self.log.info("current draw is %s Amps, voltage draw is %s Volts", self.power.battery.current, self.power.battery.voltage)

    def turn(self, degrees, power=40, ratio=-1, sleep_360=2.14):
        """
        Turn degrees anticlockwise.
        If passed negative, turn clockwise
        """
        if degrees < 0:
            self.lastTurn = "Left"
            power = -power
            degrees = abs(degrees)
        else:
            self.lastTurn = "Right"
        if degrees < 25:
            power = power / 2
            sleep_360 = sleep_360 * 2
        self.log.info("Turning %s degrees", degrees)
        self.motors[0].m0.power = power * -ratio
        self.motors[1].m1.power = power

        t_end = time.time() + sleep_360 / 360 * degrees
        while time.time() < t_end:
            self.log.info("starting %s Amps, starting %s Volts", self.power.battery.current, self.power.battery.voltage)

        self.motors[0].m0.power = 0
        self.motors[1].m1.power = 0
