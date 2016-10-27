#Just a quick note, can people who work on this code add comments, so when other people get to it they know what is going on. Thank you.


from sr.robot import *
import time

class MarkerNotFoundError(Exception): pass

class Test(Robot):#Object
    
    def __init__(self):
        print('Start Hobo init')
        super(Test, self).__init__()
        print('Robot initialised')
        while 1:
            marker = self.find_markers(max_loop=10000)[0]
            print "rot_x", marker.orientation.rot_x, "rot_y", marker.orientation.rot_y, "rot_z", marker.orientation.rot_z
            print "pol_rot_x", marker.centre.polar.rot_x, "pol_rot_z", marker.centre.polar.rot_y
    
    def find_markers(self, minimum=1, max_loop=20):
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
        
    def turn(self, degrees, power=50, ratio=-1, sleep_360=2.14):
        self.motors[0].m0.power = power*-ratio
        self.motors[0].m1.power = power
        time.sleep(sleep_360/360*degrees)
        self.motors[0].m0.power = 0
        self.motors[0].m1.power = 0
        
        
    forwards(1)
    turn(180)
    forwards(1)
    
    def goto_marker(
        
        
        
        
if __name__ == "__main__":
    Test()