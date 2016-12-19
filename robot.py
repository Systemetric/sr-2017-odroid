# Just a quick note, can people who work on this code add comments, so when other people get to it they know what is going on. Thank you.

from sr.robot import *

import collections
import time
import math
from math import sin, cos, asin, pi, sqrt
import serial
import logging
import functools
from mbed_link import StepperMotors

import strategies


# All angles should be in radians.
Vector = collections.namedtuple("Vector", ["distance", "angle"])
# The distance from the centre of rotation to the webcam.
cube_width = 0.245  # +/- 0.010
webcam_offset = 0.245  # The robot is ~2 cubes long.


class MarkerNotFoundError(Exception): pass


class Test(Robot):
    """
    A path-finding robot.
    """

    def __init__(self):
        self.init_logger()
        self.strategy = 0
        # Please use `log.debug`, `log.info`, `log.warning` or `log.error` instead of `print`

        self.log.info('Start TobyDragon init')
        super(Test, self).__init__()
        self.log.info('Robot initialised')
        self.wheels = StepperMotors(self.log)

        strategies.strategies[self.strategy](self)

        # while True:
        #     self.log.info("Start goto cube.")
        #     marker = self.find_markers()[0]
        #     self.log.debug('marker.centre.polar.rot_y = %s', marker.centre.polar.rot_y)  # The angle the marker is from the robot
        #     self.log.debug('marker.orientation.rot_y = %s', marker.orientation.rot_y)  # The rotation of the marker
        #     self.faceMarker(marker)
        #     time.sleep(2)
        #     self.turnParallelToMarker()
        #     time.sleep(2)
        #     self.turnPerpendicularToFaceMarker()
        #     time.sleep(2)
        #     self.moveToCube()
        #     time.sleep(2)

    def correct_for_webcam_horizontal_placement(self, vec):
        # type: (Vector) -> Vector
        """
        Correct for the fact that the webcam is not located at the robot's
        centre of rotation.

        Takes a vector from the point of view of the webcam, modifies it to be
        from the point of view of the robot's centre of rotation and returns it.

        See <https://hillsroadrobotics.slack.com/files/anowlcalledjosh/F3GHUJF8D/office_lens_20161219-122405.jpg>
        or <http://imgur.com/kchEXdP> for a graphical description of how this
        works.
        """
        d = vec.distance
        alpha = vec.angle
        r = webcam_offset
        m = sqrt(d**2 + r**2 - 2 * d * r * cos(pi - alpha))
        gamma = asin(d * sin(pi - alpha) / m)
        return Vector(distance=m, angle=gamma)

    def correct_for_cube_marker_placement(self, vec, beta):
        # type: (Vector) -> Vector
        """
        Correct for the fact that cube markers are on the edge of the cube, so
        the centre of the marker isn't the same as the centre of the cube.

        This function requires the angle beta to be passed to it, since it can't
        be inferred from a vector of d and alpha. It should usually be set to
        marker.orientation.rot_y.

        See <https://hillsroadrobotics.slack.com/files/anowlcalledjosh/F3GHUJF8D/office_lens_20161219-122405.jpg>
        or <http://imgur.com/kchEXdP> for a graphical description of how this
        works.
        """
        d = vec.distance
        alpha = vec.angle
        r = cube_width / 2
        m = sqrt(d**2 + r**2 - 2 * d * r * cos(pi - beta))
        gamma = asin(r * sin(pi - beta) / m)
        return Vector(distance=m, angle=gamma + alpha)

    def faceMarker(self, marker):
        """
        Given a marker, point towards it.
        """
        self.log.info('Turning to face marker')
        self.log.debug('marker.centre.polar.rot_y = %s', marker.centre.polar.rot_y)  # The angle the marker is from the robot
        self.log.debug('marker.orientation.rot_y = %s', marker.orientation.rot_y)  # The rotation of the marker
        turnOne = marker.centre.polar.rot_y  # Turns the robot to face the marker
        self.wheels.turn(turnOne)
        marker = self.find_markers()[0]
        while abs(marker.centre.polar.rot_y) > 5.0:  # If the robot is not facing the marker
            marker = self.find_markers(delta_angle=10)[0]
            self.log.debug('Not correctly aligned')
            self.log.debug('marker.centre.polar.rot_y = %s', marker.centre.polar.rot_y)  # The angle the marker is from the robot
            turnOne = marker.centre.polar.rot_y  # Turns the robot to face the marker
            self.wheels.turn(turnOne)

    def turnParallelToMarker(self):
        """
        After facing marker, change direction so parallel to marker
        Move so in a straight line with marker.
        """
        self.log.info('Turning parallel to marker')
        marker = self.find_markers()[0]
        self.log.debug('marker.centre.polar.rot_y = %s', marker.centre.polar.rot_y)  # The angle the marker is from the robot
        self.log.debug('marker.orientation.rot_y = %s', marker.orientation.rot_y)  # The rotation of the marker
        if self.wheels.lastTurn == 'Left':  # Turns the robot left or right to be perpendicular to the marker
            turnTwo = -(90 - abs(marker.orientation.rot_y))
            self.log.debug('Turn two, left %s degrees', turnTwo)
        else:
            turnTwo = (90 - abs(marker.orientation.rot_y))
            self.log.debug('Turn two, right %s degrees', turnTwo)

        lengthOne = marker.dist
        self.log.debug('Length from marker to robot: %s', marker.dist)
        lengthTwo = (lengthOne) * math.sin(math.radians(marker.orientation.rot_y))  # Works out the length to travel to be perpendicualr to the marker
        self.log.debug('Length two: %s', lengthTwo)
        self.wheels.turn(turnTwo)
        time.sleep(2)
        self.log.info('Moving to be perpendicular to marker')
        self.wheels.forwards(lengthTwo)

    def turnPerpendicularToFaceMarker(self):
        """
        Given a turn of 90 degrees would face towards cube, work out which direction and turn.
        """
        if self.wheels.lastTurn == 'Left':  # Turns left or right depending on which side the marker is
            turnThree = 90
            self.log.debug('right turn %s degrees', turnThree)
        else:
            turnThree = -90
            self.log.debug('left turn %s degrees', turnThree)
        self.log.info('Turning to face marker')
        self.wheels.turn(turnThree)
        marker = self.find_markers()[0]
        while abs(marker.centre.polar.rot_y) > 5.0:  # If the robot is not facing the marker
            marker = self.find_markers(delta_angle=10)[0]
            self.log.debug('Not correctly aligned')
            self.log.debug('marker.centre.polar.rot_y = %s', marker.centre.polar.rot_y)  # The angle the marker is from the robot
            turnOne = marker.centre.polar.rot_y  # Turns the robot to face the marker
            self.wheels.turn(turnOne)

    def moveToCube_dc(self, minimum_check_distance=0.9):
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
            # Sasha Shtyrov has added this code and extra debug information, to make sure the robot actually reaches the cube and drives over it.
            self.log.debug("Marker distance: %s", marker.dist)
            self.log.debug("Marker size: %s", marker.info.size)
            if marker.dist < minimum_check_distance:  # If robot is close to the cube, just drive over it
                self.log.debug("Driving straight towards cube")
                self.wheels.forwards(distanceFromCube + 1)
                break
            self.wheels.forwards(distanceFromCube - (distanceFromCube / 3))
            while abs(marker.centre.polar.rot_y) > 5.0:  # If the robot is not facing the marker (over 5 degrees off)
                marker = self.find_markers()[0]
                self.log.debug('Not correctly aligned')
                self.log.debug('marker.centre.polar.rot_y = %s', marker.centre.polar.rot_y)  # The angle the marker is from the robot
                turnOne = marker.centre.polar.rot_y  # Turns the robot to face the marker
                # self.turn(turnOne)

    def moveToCube(self, check_at=1, max_safe_distance=1.5):
        """
        Given that the robot is facing a cube, move to finish on top of the cube.

        check_at is the distance from the cube to check whether we're still driving towards the cube
        max_safe_distance is the maximum distance from the cube before check_at comes into effect
        check_at must be less that max_safe_distance
        """
        cube_size = 0.255
        marker = self.find_markers()[0]
        distance_to_cube = marker.dist
        self.log.debug("Cube is %s metres away", distance_to_cube)
        if distance_to_cube < max_safe_distance:
            self.log.debug("Moving straight to cube, since distance (%s) is under max safe distance (%s)", distance_to_cube, max_safe_distance)
            self.wheels.forwards(distance_to_cube + cube_size)
        else:
            # We need to check where we are once we're check_at distance from the cube
            distance_to_move = distance_to_cube - check_at
            self.log.debug("Cube is %s metres away, moving %s metres then checking", distance_to_cube, distance_to_move)
            self.wheels.forwards(distance_to_move)
            marker = self.find_markers()[0]
            while abs(marker.centre.polar.rot_y) > 1.0:  # If the robot is over 1 degrees off:
                self.log.debug("Not correctly aligned")
                self.log.debug("We're %s degrees off, correcting...", marker.centre.polar.rot_y)  # The angle the marker is from the robot
                self.wheels.turn(marker.centre.polar.rot_y)
                marker = self.find_markers()[0]
            self.log.debug("Moving the rest of the way to the cube (%s + cube_size (0.255)); this should be about 1.255 metres", marker.dist)
            self.wheels.forwards(marker.dist + cube_size)
        self.log.debug("Done moving to cube")

    def find_markers(self, minimum=1, max_loop=10, delta_angle=20):
        """
        Find at least minimum markers.
        Try max_loop attempts for each direction.

        Scans at 0. (relative)
        If fail, scan -20.
        If fail, scan +40.
        If fail, while not scanned 360,
            scan +20
        If fail, log an error but return `markers` anyway
        """

        # Scan 0.
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
        if markers == 0:
            self.log.error("Markers (minimum %s) not found with %s loops per direction", minimum, max_loop)
        return markers

    def lookForMarkers(self, max_loop=float("inf"), sleep_time=0.5):
        """
        Look for markers.
        if none found within max_loop, return []
        """
        self.log.info("Looking for markers...")
        time.sleep(sleep_time)  # Rest so camera can focus
        markers = self.see()
        i = 0
        while i <= max_loop and len(markers) == 0:
            self.log.debug("Cannot see a marker")
            i += 1
            markers = self.see()
        return markers

    def routeNine(self):  # Code for moving along a route to pick up cubes  B2,C,A1
        self.wheels.forwards(3)          # Move along the edge of arena //Should be 3.5m but for testing its 3m
        self.wheels.turn(90)             # Turn to face 1st marker/cube: B2
        marker = self.find_markers()[0]  # Look for the marker/cube: B2 to get distance and rotation
        self.faceMarker(marker)          # Make sure robot is facing cube
        self.moveToCube(marker)          # Drive towards marker/cube:B2
        marker = self.find_markers()[0]  # Look for the marker/cube:C which should be in front of the robot
        self.faceMarker(marker)          # Make sure robot is facing marker/cube:C
        self.moveToCube(marker)          # Move towards cube/marker: C
        self.wheels.turn(135)            # Turn to face cube/marker: A1
        marker = self.find_markers()[0]  # Look for the marker/cube: A1 to get distance and rotation
        self.faceMarker(marker)          # Make sure robot is facing marker/cube: A1
        self.moveToCube(marker)          # Move towards cube/marker: A1
        self.wheels.forwards(3.5)        # Move HOME

    def find_specific_markers(self, marker_type, delta_angle=20):
        """
        Searches for markers in a similar way to find_markers().
        """
        self.log.debug("Finding marker of type %s", marker_type)
        self.log.warn("Not yet implemented")
        # The maximum number of times to check for a marker from one angle.
        max_loop = 5
        # Get a list of markers that are of the requested type.
        markers = filter(lambda m: m.info.marker_type == marker_type, self.lookForMarkers(max_loop=max_loop))
        if len(markers) == 0:
            # Turn slightly left in case we're facing just right of the marker.
            self.wheels.turn(-delta_angle)
            i = 0
            # Search for markers in a full circle
            while i <= 360 and len(markers) == 0:
                markers = filter(lambda m: m.info.marker_type == marker_type, self.lookForMarkers(max_loop=max_loop))
                if len(markers) == 0:
                    self.wheels.turn(delta_angle)
                    i += delta_angle
        return markers

    def init_logger(self):
        """
        Initialise logger.
        """
        self.log = logging.getLogger(__name__)
        self.log.setLevel(logging.DEBUG)
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.DEBUG)
        # Example: "filename:42 - do_stuff() - INFO: stuff happened"
        formatter = logging.Formatter('%(module)s:%(lineno)d - %(funcName)s() - %(levelname)s: %(message)s')
        console_handler.setFormatter(formatter)
        self.log.addHandler(console_handler)


if __name__ == "__main__":
    Test()
