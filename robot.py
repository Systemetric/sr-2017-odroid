# Just a quick note, can people who work on this code add comments, so when other people get to it they know what is going on. Thank you.

from sr.robot import *

import time
from math import sin, cos, asin, pi, sqrt, radians, degrees, atan2
import logging
from operator import attrgetter

from mbed_link import StepperMotors
import strategies
import corrections
from vector import Vector, marker2vector


cube_width = 0.245  # +/- 0.010


class Test(Robot):
    """
    A path-finding robot.
    """

    def __init__(self):
        self.init_logger()
        self.pre_init_strategy = None
        self.strategy = "none"
        # Please use `log.debug`, `log.info`, `log.warning` or `log.error` instead of `print`

        self.log.info("Start TobyDragon init")
        super(Test, self).__init__(init=False)
        self.init()
        self.wheels = StepperMotors(self.log)
        self.log.info("Robot initialised")
        self.log.info("Battery(voltage = {}, current = {})".format(self.power.battery.voltage, self.power.battery.current))
        args = []
        if self.pre_init_strategy:
            args = strategies.strategies[self.pre_init_strategy](self)
        # Wait until the start button is pressed
        switch_state = self.wheels.get_switch_state()
        self.log.info("DIP switch is %s", switch_state)
        self.log.info("Waiting for start signal...")
        self.wait_start()
        self.log.info("Start signal recieved!")
        strategies.strategies[self.strategy](self, *args)

    def faceMarker(self, marker):
        # type: (Marker) -> None
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

    def face_cube(self, marker):
        # type: (Marker) -> float
        """
        Given a cube marker, face the centre of the cube.
        Returns the distance required to travel on top of that cube
        """
        self.log.info("Facing marker...")
        vec = marker2vector(marker)
        vec = corrections.correct_all_cube(vec, marker.orientation.rot_y)
        self.log.debug("Turning %s degrees (%s radians)", degrees(vec.angle), vec.angle)
        self.wheels.turn(degrees(vec.angle))
        return vec.distance + corrections.cube_width

    def move_to_cube(self, marker, check_at=1.0, max_safe_distance=3, angle_tolerance=1.0, distance_after=0.0):
        # type: (Marker, float, float, float) -> None
        """
        Given a cube marker, face and move to the cube.

        At check_at metres away from the cube, stop and check if we're still
        facing the right way, unless we started less than max_safe_distance away
        from the cube. "The right way" is defined as within angle_tolerance of
        the angle we should be facing.
        """
        distance = self.face_cube(marker)
        if distance <= max_safe_distance:
            self.log.debug("Moving straight to cube, since distance (%s) is under max safe distance (%s)", distance, max_safe_distance)
            self.wheels.forwards(distance+distance_after)
        else:
            # We need to check where we are once we're check_at distance from the cube
            distance_to_move = distance - corrections.cube_width - check_at
            self.log.debug("Cube is %s metres away, moving %s metres then checking", distance, distance_to_move)
            self.wheels.forwards(distance_to_move)
            while True:  # If the robot is over 1 degrees off:
                markers = self.find_markers(filter_func = lambda m: m.info.code == marker.info.code)
                if not markers:
                    return False
                marker = markers[0]
                vec = marker2vector(marker)
                vec = corrections.correct_all_cube(vec, marker.orientation.rot_y)
                if abs(degrees(vec.angle)) <= angle_tolerance:
                    break
                self.log.debug("Not correctly aligned")
                self.log.debug("We're %s degrees off, correcting...", degrees(vec.angle))  # The angle the marker is from the robot
                self.wheels.turn(degrees(vec.angle))
            self.log.debug("Moving the rest of the way to the cube (%s + cube_size (0.255)); this should be about 1.255 metres", vec.distance)
            self.wheels.forwards(vec.distance+distance_after)
        self.log.debug("Done moving to cube")
        return True

    def find_closest_marker(self, marker_type):
        # type: (...) -> Marker
        """
        Find and return the closest marker of a given marker type.
        
        If no markers can be found, an IndexError will be raised.
        """
        self.log.info("Finding closest marker of type %s", marker_type)
        markers = [m for m in self.find_markers(filter_func=lambda marker: marker.info.marker_type == marker_type)]
        return sorted(markers, key=attrgetter("dist"))[0]

    def find_marker_approx_position(self, marker_type, dist, dist_tolerance = 0.25):
        """
        Find and return a list of markers at an approximate location.

        The list may be empty, in which case no markers could be seen at that distance.
        """
        self.log.info("Finding marker of type %s approximately %s metres away, give or take %s metres", marker_type, dist, dist_tolerance)
        markers = [m for m in self.lookForMarkers(max_loop=5) if m.info.marker_type == marker_type and dist - dist_tolerance <= m.dist <= dist + dist_tolerance]
        self.log.info("Found %s markers matching criteria", len(markers))
        return markers

    def find_markers(self, minimum=1, max_loop=10, delta_angle=20, filter_func=lambda markers: markers):
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
        markers = filter(filter_func, self.lookForMarkers(max_loop=max_loop))
        if len(markers) >= minimum:
            # If found correct number of markers, stop and return them
            return markers
        else:
            # If the robot cannot see a marker
            self.log.debug("Searching for markers... (direction = %s)", -delta_angle)
            self.wheels.turn(-delta_angle)
            markers = filter(filter_func, self.lookForMarkers(max_loop=max_loop))
            if len(markers) >= minimum:
                return markers
            else:
                self.wheels.turn(delta_angle * 2)
                i = delta_angle
                while i <= 360:
                    # Continue scanning in a circle - probably not in a simple arc
                    self.log.debug("Searching for markers... (direction = %s)", i)
                    markers = filter(filter_func, self.lookForMarkers(max_loop=max_loop))
                    self.wheels.turn(delta_angle)
                    if len(markers) >= minimum:
                        return markers
                    i += delta_angle
        # Current direction is ~360 (no change)
        self.log.error("Markers (minimum %s) not found with %s loops per direction", minimum, max_loop)
        return markers

    def lookForMarkers(self, max_loop=float("inf"), sleep_time=0.5):
        """
        Look for markers.
        if none found within max_loop, return []
        """
        self.log.info("Looking for markers with %s attempts...", max_loop)
        time.sleep(sleep_time)  # Rest so camera can focus
        markers = self.see()
        i = 0
        while i <= max_loop and len(markers) == 0:
            self.log.debug("Cannot see a marker")
            i += 1
            markers = self.see()
        return markers

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

    @staticmethod
    def get_wall_marker_coords(marker_id):
        direction, wall = divmod(marker_id, 7)
        decrement, swap = divmod(direction, 2)
        if decrement:
            wall = 6-wall
        wall += 1
        rtn = [wall, 8 * (7 <= marker_id <= 20)]
        if swap:
            rtn = rtn[::-1]
        return rtn
    
    def get_team_corner_coords(self):
        return [[0, 0], [8, 0], [8, 8], [0, 8]][self.zone]

    def get_vec_to_corner(self, marker_id):
        marker_pos = self.get_wall_marker_coords(marker_id)
        corner_pos = self.get_team_corner_coords()
        deltas = marker_pos[0]-corner_pos[0], marker_pos[1]-corner_pos[1]
        distance = sqrt(deltas[0]**2 + deltas[1]**2)
        angle = atan2(deltas[1], deltas[0])
        angle += pi - (1+(marker_id // 7))*(pi/2)
        angle %= 2*pi
        angle -= pi
        return Vector(distance=distance, angle=angle)

    lookup_distances = {
        0: 1,
        1: 2,
        2: 3,
        3: 4,
        4: 5,
        5: 6,
        6: 7,
        # TODO: Pythagoras
        21: 7,
        22: 6,
        23: 5,
        24: 4,
        25: 3,
        26: 2,
        27: 1
    }

    def get_vec_to_corner2(self, vec, beta, marker_id):
        d = vec.distance
        alpha = vec.angle
        l = self.lookup_distances[(marker_id - self.zone*7) % 28]
        m = sqrt(d**2 + l**2 - 2*d*l*cos(pi/2 - beta))
        gamma = asin(l*sin(pi/2 - beta)/m) - alpha
        return Vector(distance=m, angle=gamma)

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
