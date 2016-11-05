#Just a quick note, can people who work on this code add comments, so when other people get to it they know what is going on. Thank you.


from sr.robot import *
import time
import math

class MarkerNotFoundError(Exception): pass
lastTurn = ''

class Test(Robot):#Object
    
    def __init__(self):
        print('Start Hobo init')
        super(Test, self).__init__()
        print('Robot initialised')
        while True:
            marker = self.find_markers(max_loop=2000)[0]
            if marker > 0:
                #self.goTo(marker)
                print('marker.centre.polar.rot_y = ', marker.centre.polar.rot_y)#The angle the marker is from the robot
                print('marker.orientation.rot_y = ', marker.orientation.rot_y)# The rotation of the marker
                self.faceMarker(marker)
                self.turnParallelToMarker()
                time.sleep(2)
                
        #while 1:
            #marker = self.find_markers(max_loop=10000)[0]
            #print "rot_x", marker.orientation.rot_x, "rot_y", marker.orientation.rot_y, "rot_y", marker.orientation.rot_z# marker rotation
            #print "pol_rot_x", marker.centre.polar.rot_x, "pol_rot_z", marker.centre.polar.rot_y#Marker rotation from robot
            
    
    def faceMarker(self, marker):
        print('Turning to face marker')
        print('marker.centre.polar.rot_y = ', marker.centre.polar.rot_y)#The angle the marker is from the robot
        print('marker.orientation.rot_y = ', marker.orientation.rot_y)# The rotation of the marker
        turnOne = marker.centre.polar.rot_y #Turns the robot to face the marker
        self.turn(turnOne)
        if math.fabs(marker.centre.polar.rot_y) > 5.0: #If the robot is not facing the marker
            marker = self.find_markers(max_loop=2000)[0]
            print('Not correctly aligned')
            print('marker.centre.polar.rot_y = ', marker.centre.polar.rot_y)#The angle the marker is from the robot
            turnOne = marker.centre.polar.rot_y #Turns the robot to face the marker
            self.turn(turnOne)
    
    def turnParallelToMarker(self):
        print('Turning parallel to marker') 
        marker = self.find_markers(max_loop=2000)[0]
        if lastTurn == 'Left':#Turns the robot left or right to be perpendicular to the marker
            turnTwo = -(90 -  math.fabs(marker.orientation.rot_y))
            print('Turn two, left', turnTwo)
        else:
            turnTwo = (90 -  math.fabs(marker.orientation.rot_y))
            print('Turn two, right', turnTwo)
                
        lengthOne = marker.dist
        print('Length from marker to robot', marker.dist)
        lengthTwo = (lengthOne) * math.degrees(math.sin(math.radians(marker.orientation.rot_y)))#Works out the length to travel to be perpendicualr to the marker
        print('Length two', lengthTwo)
        self.turn(turnTwo)
        time.sleep(2)
        print('Moving to be perpendicular to marker')
        self.forwards(lengthTwo)
        time.sleep(2)
        
    def turnPerpendicularToFaceMarker(self)
        if  lastTurn == 'Left':#Turns lefts or right depending on which side the marker is
            turnThree = 90
            print('right turn', turnThree)
        else:
            turnThree = -90
            print('left turn', turnThree)
        print('Turning to face marker')
        self.turn(turnThree)
        time.sleep(2)
        
    
    def goTo(self, marker):
        lengthOne = float
        turnThree = float
        
        
        
        
        
        time.sleep(2)
        
        markers = self.see()
        print('Looking for marker')
        while markers.count == 0:
            print('Looking for markers')
            self.turn(30)
            time.sleep(0.5)
        if markers[0].dist > 1:#Moves forward so the robot is 1m away from the marker
            lengthOne = markers[0].dist - 1
            print('New length from marker', markers[0].dist)
            self.forwards(lengthOne)
        markers = self.see()
        while markers.count == 0:
            print('Looking for markers')
            self.turn(30)
            time.sleep(0.5) 
        print('Marker rotation', markers[0].centre.polar.rot_y)
        while markers[0].centre.polar.rot_y != 0:
            print('Not centered')
            self.turn(markers[0].centre.polar.rot_y)
            time.sleep(0.5)
        lengthOne = markers[0].dist
        self.forwards(lengthOne)
        return
    
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
        distance = math.fabs(distance)
        power = speed * speed_power
        sleep_time = distance / speed
        print "Distance", distance, "ST",sleep_time, "P", power, "PR", power*ratio 
        self.motors[0].m0.power = power*ratio
        self.motors[0].m1.power = power
        time.sleep(sleep_time)
        self.motors[0].m0.power = 0
        self.motors[0].m1.power = 0
        
    def turn(self, degrees, power=50, ratio=-1, sleep_360=2.14):
        if degrees < 0:
            global lastTurn
            lastTurn = 'Left'
            power = -(power)
            degrees = math.fabs(degrees)
        else:
            global lastTurn 
            lastTurn = 'Right'
        if degrees < 25:
            power = power / 2
            sleep_360 = sleep_360 * 2
        print "Turn",degrees, "Power", power
        self.motors[0].m0.power = power*-ratio
        self.motors[0].m1.power = power
        time.sleep(sleep_360/360*degrees)
        
        self.motors[0].m0.power = 0
        self.motors[0].m1.power = 0
        
                
    

        
if __name__ == "__main__":
    Test()


