#Just a quick note, can people who work on this code add comments, so when other people get to it they know what is going on. Thank you.


from sr.robot import *
import time
import math
import serial
import logging
from mbed_link import StepperMotors

class MarkerNotFoundError(Exception): pass

class Test(Robot):#Object
    """
    A path-finding robot.
    """
    
    def __init__(self):
        self.init_logger()
        # Please use `log.debug`, `log.info`, `log.warning` or `log.error` instead of `print`

        self.log.info('Start TobyDragon init')
        super(Test, self).__init__()
        self.log.info('Robot initialised')
        self.wheels = StepperMotors(self.log)
        while True:
            self.log.info("Start goto cube.")
            marker = self.find_markers()[0]
            self.log.debug('marker.centre.polar.rot_y = %s', marker.centre.polar.rot_y) #The angle the marker is from the robot
            self.log.debug('marker.orientation.rot_y = %s', marker.orientation.rot_y) #The rotation of the marker
            self.faceMarker(marker)
            time.sleep(2)
            self.turnParallelToMarker()
            time.sleep(2)
            self.turnPerpendicularToFaceMarker()
            time.sleep(2)
            self.moveToCube()
            time.sleep(2)

    
    def faceMarker(self, marker):
        """
        Given a marker, point towards it.
        """
        self.log.info('Turning to face marker')
        self.log.debug('marker.centre.polar.rot_y = %s', marker.centre.polar.rot_y) #The angle the marker is from the robot
        self.log.debug('marker.orientation.rot_y = %s', marker.orientation.rot_y) #The rotation of the marker
        turnOne = marker.centre.polar.rot_y #Turns the robot to face the marker
        self.wheels.turn(turnOne)
        marker = self.find_markers()[0]
        while math.fabs(marker.centre.polar.rot_y) > 5.0: #If the robot is not facing the marker
            marker = self.find_markers(delta_angle=10)[0]
            self.log.debug('Not correctly aligned')
            self.log.debug('marker.centre.polar.rot_y = %s', marker.centre.polar.rot_y) #The angle the marker is from the robot
            turnOne = marker.centre.polar.rot_y #Turns the robot to face the marker
            self.wheels.turn(turnOne)
    

    def turnParallelToMarker(self):
        """
        After facing marker, change direction so parallel to marker
        Move so in a straight line with marker.
        """
        self.log.info('Turning parallel to marker') 
        marker = self.find_markers()[0]
        self.log.debug('marker.centre.polar.rot_y = %s', marker.centre.polar.rot_y)#The angle the marker is from the robot
        self.log.debug('marker.orientation.rot_y = %s', marker.orientation.rot_y)# The rotation of the marker
        if self.wheels.lastTurn == 'Left':#Turns the robot left or right to be perpendicular to the marker
            turnTwo = -(90 -  math.fabs(marker.orientation.rot_y))
            self.log.debug('Turn two, left %s degrees', turnTwo)
        else:
            turnTwo = (90 -  math.fabs(marker.orientation.rot_y))
            self.log.debug('Turn two, right %s degrees', turnTwo)
                
        lengthOne = marker.dist
        self.log.debug('Length from marker to robot: %s', marker.dist)
        lengthTwo = (lengthOne) * math.sin(math.radians(marker.orientation.rot_y))#Works out the length to travel to be perpendicualr to the marker
        self.log.debug('Length two: %s', lengthTwo)
        self.wheels.turn(turnTwo)
        time.sleep(2)
        self.log.info('Moving to be perpendicular to marker')
        self.wheels.forwards(lengthTwo)
        

    def turnPerpendicularToFaceMarker(self):
        """
        Given a turn 0f 90 degrees would face towards cube,
        Work out which direction and turn.
        """
        if  self.wheels.lastTurn == 'Left':# Turns left or right depending on which side the marker is
            turnThree = 90
            self.log.debug('right turn %s degrees', turnThree)
        else:
            turnThree = -90
            self.log.debug('left turn %s degrees', turnThree)
        self.log.info('Turning to face marker')
        self.wheels.turn(turnThree)
        marker = self.find_markers()[0]
        while math.fabs(marker.centre.polar.rot_y) > 5.0: #If the robot is not facing the marker
            marker = self.find_markers(delta_angle=10)[0]
            self.log.debug('Not correctly aligned')
            self.log.debug('marker.centre.polar.rot_y = %s', marker.centre.polar.rot_y)#The angle the marker is from the robot
            turnOne = marker.centre.polar.rot_y #Turns the robot to face the marker
            self.wheels.turn(turnOne)
   

    def moveToCube(self, minimum_check_distance=0.9):
        """
        Given that robot is facing cube,
        Move towards cube in a straight line,
        Rechecking straight every 2/3 distance.
        minimum_check_distance is the distance to the cube below which the robot does not stop to recheck straightness.
        """
        marker = self.find_markers()[0]
        distanceFromCube = marker.dist
        while distanceFromCube > 0.5:
            self.log.info('Moving towards marker')
            marker = self.find_markers()[0]
            distanceFromCube = marker.dist
            #Sasha Shtyrov has added this code and extra debug information, to make sure the robot actually reaches the cube and drives over it.
            self.log.debug("Marker distance: %s", marker.dist)
            self.log.debug("Marker size: %s", marker.info.size)
            if marker.dist < minimum_check_distance: #If robot is close to the cube, just drive over it
                self.wheels.forwards(distanceFromCube + 1)
                break
            self.wheels.forwards(distanceFromCube - (distanceFromCube / 3))
            while math.fabs(marker.centre.polar.rot_y) > 5.0: #If the robot is not facing the marker
                marker = self.find_markers()[0]
                self.log.debug('Not correctly aligned')
                self.log.debug('marker.centre.polar.rot_y = %s', marker.centre.polar.rot_y)#The angle the marker is from the robot
                turnOne = marker.centre.polar.rot_y #Turns the robot to face the marker
                #self.turn(turnOne)
        
        
    def find_markers(self, minimum=1, max_loop=10, delta_angle=20):
        """
        Find at least minimum markers.
        Try max_loop attempts for each direction.

        Scans at 0. (relative)
        If fail, scan -20.
        If fail, scan +40.
        If fail, while not scanned 360,
            scan +20
        If fail raise MarkerNotFoundError
        """

        self.log.info("Searching for markers...")
        #Scan 0.
        self.log.debug("Searching for markers... (direction = 0)")
        markers = self.lookForMarkers(max_loop=max_loop)
        if len(markers) >= minimum:
            # If found correct number of markers, stop and return them
            return markers
        else:
            
            # If the robot cannot see a marker
            self.log.debug("Searching for markers... (direction = %s)", -delta_angle)
            self.wheels.turn(-delta_angle)
            markers = self.lookForMarkers(max_loop=max_loop)
            if len(markers) >= minimum:
                i = 361
                return markers
            else:                
                self.wheels.turn(delta_angle * 2)
                i = delta_angle
            while i <= 360:
            # Continue scanning in a circle - probably not in a simple arc
                self.log.debug("Searching for markers... (direction = %s)", i)
                markers = self.lookForMarkers(max_loop=max_loop)
                self.wheels.turn(delta_angle)
                if len(markers) >= minimum:
                    i = 361
                    return markers
                    
                else:
                    i += delta_angle
            
        # Current direction is ~360 (no change)
        if markers ==0:
            
            self.log.error("Marker(s) (minimum %s) not found with %s loops per direction", minimum, max_loop)
        return markers
        

    def lookForMarkers(self, max_loop=float("inf"), sleep_time=0.5):
        """
        Look for markers.
        if none found within max_loop, return []
        """
        self.log.info("Looking for markers...")
        time.sleep(sleep_time)# Rest so camera can focus
        markers = self.see()
        i = 0
        while i <= max_loop and len(markers) == 0:
            self.log.debug("Cannot see a marker")
            i += 1
            markers = self.see()
        return markers
        
    def routeNine(self):      #Code for moving along a route to pick up cubes  B2,C,A1
        self.wheels.forwards(3)      #Move along the edge of arena //Should be 3.5m but for testing its 3m
        self.wheels.turn(90)         #Turn to face 1st marker/cube: B2
        marker = self.find_markers()[0]#Look for the marker/cube: B2 to get distance and rotation
        self.faceMarker(marker)        #Make sure robot is facing cube
        self.moveToCube(marker)        #Drive towards marker/cube:B2
        marker = self.find_markers()[0]#Look for the marker/cube:C which should be in front of the robot
        self.faceMarker(marker)        #Make sure robot is facing marker/cube:C
        self.moveToCube(marker)        #Move towards cube/marker: C
        self.wheels.turn(135)                 #Turn to face cube/marker: A1
        marker = self.find_markers()[0]#Look for the marker/cube: A1 to get distance and rotation
        self.faceMarker(marker)        #Make sure robot is facing marker/cube: A1
        self.moveToCube(marker)        #Move towards cube/marker: A1
        self.wheels.forwards(3.5)             #Move HOME
        

    def init_logger(self):
        """
        Initialise logger.
        """
        self.log = logging.getLogger(__name__)
        self.log.setLevel(logging.DEBUG)
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(name)s - %(levelname)s - %(message)s')
        console_handler.setFormatter(formatter)
        self.log.addHandler(console_handler)
        
        
if __name__ == "__main__":
    Test()


