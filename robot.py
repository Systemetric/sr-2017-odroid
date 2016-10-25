from sr.robot import *
import time

class MarkerNotFoundError(Exception): pass

class Test(Robot):
    
    def __init__(self):
        print('Start Hobo init')
        super(Test, self).__init__()
        print('Robot initialised')
        self.forwards(2)

    
    def find_markers(self, minimum=1, max_loop=10):
        cur = 0
        markers = []
        while len(markers) < minimum:
            cur += 1
            print("Searching for markers...")
            markers = self.see()
            if cur == max_loop:
                raise MarkerNotFoundError("Marker (minimum {}) not found after {} loops".format(minimum, max_loop))
        return markers
        
    def forwards(self, distance, speed=0.75, ratio=-1.05, speed_power = 77):        
        power = speed * speed_power
        self.motors[0].m0.power = power*ratio
        self.motors[0].m1.power = power
        sleep_time = distance / speed
        print "ST",sleep_time, "P", power, "PR", power*ratio
        time.sleep(sleep_time)
        print "Slept"
        self.motors[0].m0.power = 0
        self.motors[0].m1.power = 0

        
if __name__ == "__main__":
    Test()