"""Generic code used in many strategies.

This file is part of the code for the Hills Road/Systemetric entry to
the 2017 Student Robotics competition "Easy as ABC".
"""


from __future__ import division

from sr.robot import *

import time
from math import sqrt
import logging
from operator import attrgetter

try:
    # noinspection PyUnresolvedReferences
    from typing import Callable, List
except ImportError:
    pass

from mbed_link import Mbed, MovementInterruptedError
import strategies
import corrections
from trig import sind, cosd, asind
from vector import Vector, marker2vector


class CompanionCube(Robot):
    """
    A path-finding robot.
    """

    def __init__(self):
        # Please use `log.debug`, `log.info`, `log.warning` or `log.error` instead of `print`
        self.init_logger()

        self.strategy = "b c a"
        args = []
        kwargs = {"opposite_direction": False}
        self.routeChange = False

        self.log.info("Start TobyDragon init")
        super(CompanionCube, self).__init__(init=False)
        self.init()
        self.wheels = Mbed(self.log)
        self.log.info("Robot initialised")
        self.log.info("Battery(voltage = %s, current = %s)", self.power.battery.voltage, self.power.battery.current)
        switch_state = self.wheels.get_switch_state()
        self.log.info("DIP switch is %s", switch_state)
        self.log.info("Waiting for start signal...")
        self.wait_start()
        self.log.info("Start signal recieved!")
        strategies.strategies[self.strategy](self, *args, **kwargs)
        self.log.info("Strategy exited.")

    def faceMarker(self, marker):
        # type: (Marker) -> None
        """
        Given a marker, point towards it.
        """
        self.log.info("faceMarker is DEPRECATED. Turning to face marker")
        self.log.debug("marker.centre.polar.rot_y = %s", marker.centre.polar.rot_y)  # The angle the marker is from the robot
        self.log.debug("marker.orientation.rot_y = %s", marker.orientation.rot_y)  # The rotation of the marker
        turnOne = marker.centre.polar.rot_y  # Turns the robot to face the marker
        self.wheels.turn(turnOne)
        marker = self.find_markers()[0]
        while abs(marker.centre.polar.rot_y) > 5.0:  # If the robot is not facing the marker
            marker = self.find_markers(delta_angle=10)[0]
            self.log.debug("Not correctly aligned")
            self.log.debug("marker.centre.polar.rot_y = %s", marker.centre.polar.rot_y)  # The angle the marker is from the robot
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
        self.log.debug("Turning %s degrees", vec.angle)
        self.wheels.turn(vec.angle)
        return vec.distance + corrections.cube_width

    def move_to_cube(self, marker, crash_continue=False, check_at=1.0, max_safe_distance=3, angle_tolerance=1.0, distance_after=0.0):
        # type: (Marker, float, float, float) -> None
        """
        Given a cube marker, face and move to the cube.

        At check_at metres away from the cube, stop and check if we're still
        facing the right way, unless we started less than max_safe_distance away
        from the cube. "The right way" is defined as within angle_tolerance of
        the angle we should be facing.
        """
        distance = self.face_cube(marker)
        move = self.wheels.move
        if crash_continue:
            move = self.move_continue
        if distance <= max_safe_distance:
            self.log.debug("Moving straight to cube, since distance (%s) is under max safe distance (%s)", distance, max_safe_distance)
            try:
                move(distance+distance_after)
            except MovementInterruptedError:
                return 'Crash'
        else:
            # We need to check where we are once we're check_at distance from the cube
            distance_to_move = distance - corrections.cube_width - check_at
            self.log.debug("Cube is %s metres away, moving %s metres then checking", distance, distance_to_move)
            try:
                move(distance_to_move)
            except MovementInterruptedError:
                return 'Crash'
            while True:  # If the robot is over 1 degrees off:
                markers = self.find_markers(filter_func=lambda m: m.info.code == marker.info.code)
                if not markers:
                    return 'Cant see'
                marker = markers[0]
                vec = marker2vector(marker)
                vec = corrections.correct_all_cube(vec, marker.orientation.rot_y)
                if abs(vec.angle) <= angle_tolerance:
                    break
                self.log.debug("Not correctly aligned")
                self.log.debug("We're %s degrees off, correcting...", vec.angle)  # The angle the marker is from the robot
                self.wheels.turn(vec.angle)
            self.log.debug("Moving the rest of the way to the cube (%s + cube_size (0.255)); this should be about 1.255 metres", vec.distance)
            try:
                move(vec.distance+distance_after)
            except MovementInterruptedError:
                return 'Crash'
        self.log.debug("Done moving to cube")
        return 'Ok'

    def move_continue(self, distance):
        """
        Attempt to continuously move a distance, retrying if required.
        """
        try:
            self.wheels.move(distance)
        except MovementInterruptedError:
            while True:
                self.log.debug("Failed to move %sm. Attempting to continue", distance)
                time.sleep(1)
                try:
                    self.wheels.retry()
                except MovementInterruptedError:
                    pass
                else:
                    return

    def move_home_from_A(self):
        # type: () -> None
        """Given we are at our A cube and facing roughly home, get home.

        If we can see either of the two markers inside our home area,
        we can move home accurately. Otherwise, we will blindly move
        forwards and hope we're facing in the right direction.
        """
        right_marker_code = self.zone * 7
        left_marker_code = (right_marker_code - 1) % 28
        other_codes = set(range(28))
        other_codes.remove(left_marker_code)
        other_codes.remove(left_marker_code - 1)
        other_codes.remove(right_marker_code)
        other_codes.remove((right_marker_code + 1) % 28)
        markers = sorted(self.see_markers(lambda m: m.info.marker_type == MARKER_ARENA), key=attrgetter("dist"))
        marker_codes = [m.info.code for m in markers]
        self.log.debug("Seen %s arena markers (codes: %s)", len(markers), marker_codes)

        walls = [
            list(range(0, 7)),
            list(range(7, 14)),
            list(range(14, 21)),
            list(range(21, 28))
        ]

        if left_marker_code in marker_codes or right_marker_code in marker_codes:
            if left_marker_code in marker_codes:
                self.log.debug("Can see left marker!")
                left_marker = filter(lambda m: m.info.code == left_marker_code, markers)[0]
                self.wheels.turn(left_marker.rot_y + 16)
            elif right_marker_code in marker_codes:
                self.log.debug("Can see right marker!")
                right_marker = filter(lambda m: m.info.code == right_marker_code, markers)[0]
                self.wheels.turn(right_marker.rot_y - 16)
            else:
                self.log.critical("Python is lying to us! This can't happen.")
            self.wheels.move(3.5)  # sqrt(2 * 2.5^2) = 3.5355 metres
        elif other_codes.intersection(marker_codes):
            bad_marker_codes = other_codes.intersection(marker_codes)
            self.log.warn("Other teams' codes (%s) are visible! We're probably facing into another team's corner :(", bad_marker_codes)
            self.move_home_from_other_A()
        else:
            self.log.warn("Can't see any useful arena markers (ours or theirs), driving forwards and praying...")
            self.wheels.move(3.5)

    def move_home_from_other_A(self, marker=None):
        # type: (...) -> None
        walls = [
            list(range(0, 7)),
            list(range(7, 14)),
            list(range(14, 21)),
            list(range(21, 28))
        ]

        right_marker_code = self.zone * 7
        left_marker_code = (right_marker_code - 1) % 28
        other_codes = set(range(28))
        other_codes.remove(left_marker_code)
        other_codes.remove(left_marker_code - 1)
        other_codes.remove(right_marker_code)
        other_codes.remove((right_marker_code + 1) % 28)
        our_markers = {(left_marker_code - 1) % 28, left_marker_code, right_marker_code, (right_marker_code + 1) % 28}

        # - move to 1.5 m away from a marker
        # - look at marker.orientation.rot_y and turn parallel with the marker (towards our corner)
        # - go forwards, checking every N metres that we're still parallel with the wall
        # - if we can see an arena marker in front of us, drive to 1.5 m from it and repeat/go home

        if marker is None:
            markers = sorted(self.see_markers(lambda m: m.info.marker_type == MARKER_ARENA), key=attrgetter("dist"))
            marker = markers[0]
            self.log.debug("Fixating upon marker %s, since it's closest (%s metres away)", marker.info.code, marker.dist)
        else:
            self.log.info("Fixating upon marker %s, since we were passed it (%s metres away)", marker.info.code, marker.dist)
        self.wheels.turn(marker.rot_y)  # Face the marker
        # Find the marker again
        markers = self.see_markers(predicate=lambda m: m.info.code == marker.info.code)
        if not markers:
            self.log.error("We turned to the marker and now can't see it.")  # Don't move!
            return
        marker = markers[0]
        # This is the index in `walls` of the wall we fixated upon.
        orig_marker_wall = [walls.index(wall) for wall in walls if marker.info.code in wall][0]
        # Move to 1.5 metres away from the marker
        self.log.debug("Moving to 1.5 metres from the marker")
        if marker.dist > 1.5:
            self.move_continue(marker.dist - 1.5)
        else:
            self.log.debug("We're closer than we should be (%s metres)!", marker.dist)
        markers = self.see_markers(predicate=lambda m: m.info.code == marker.info.code)
        if not markers:
            self.log.error("We moved closer to the marker (maybe) and now can't see it.")
            return
        # Move to 1.5 metres away from the wall
        self.log.debug("Moving to 1.5 metres from the WALL")
        dist = sqrt(1.5 ** 2 + 1.5 ** 2 - 2 * 1.5 * 1.5 * cosd(marker.orientation.rot_y))
        if marker.orientation.rot_y < 0:
            # turning right first, then left
            self.log.debug("Turning right first")
            angle = (180 - marker.orientation.rot_y) / 2
        else:
            # turning left first, then right
            self.log.debug("Turning left first")
            angle = (-180 - marker.orientation.rot_y) / 2
        self.wheels.turn(angle)
        self.move_continue(dist)
        self.wheels.turn(-angle)
        # We should now be 1.5 metres away from the wall, facing the marker head-on.
        self.log.debug("We should now be 1.5 metres away from the wall, facing the marker head-on.")
        markers = self.see_markers(predicate=lambda m: m.info.code == marker.info.code)
        if not markers:
            self.log.error("We moved to face the marker and now can't see it.")
            return
        # Turn parallel to the wall (see Slack for diagram, search "parallel to wall" in #brainstorming)
        if orig_marker_wall in (self.zone, (self.zone + 1) % 4):
            self.wheels.turn(90 + marker.orientation.rot_y)  # Turn left (wall on right, heading anticlockwise)
        else:
            self.wheels.turn(90 - marker.orientation.rot_y)  # Turn right (wall on left, heading clockwise)
        # We should now be facing along the wall.
        # Look for wall markers that won't vanish on us and that aren't on the wall we're moving along.
        markers = self.see_markers(predicate=lambda m: m.info.marker_type == MARKER_ARENA and m.dist <3 and m.info.code not in walls[orig_marker_wall])
        if markers:
            marker = markers[0]
        while not markers:
            self.log.debug("Can't see any matching wall markers (wall, close, not the wall we first saw), going forwards a bit.")
            self.move_continue(1)
            time.sleep(1)
            markers = self.see_markers(predicate=lambda m: m.info.marker_type == MARKER_ARENA and m.dist <3 and m.info.code not in walls[orig_marker_wall])
        self.log.debug("We see wall %s markers.", len(markers))
        if orig_marker_wall not in (self.zone, (self.zone - 1) % 4):
            # We started at a wall opposite our corner, go round again
            self.log.info("Recursing, since we need to go along another wall to get home. If this message appears more than once, something is wrong.")
            # Pass ourselves a sensible marker.
            marker = sorted(markers, key=attrgetter("dist"))[0]
            self.move_home_from_other_A(marker=marker)
            self.log.info("Finished recursing, hopefully we're home now. Returning.")
            return
        self.log.debug("We should now be facing our corner.")
        markers = self.see_markers(predicate=lambda m: m.info.marker_type == MARKER_ARENA)
        if our_markers.intersection(markers):
            self.log.debug("We can see some of our corner markers! (These ones: %s)", our_markers.intersection(markers))
        else:
            self.log.warn("We can't see any of our corner markers, but we should be able to (we see these: %s).", len(markers))
        # TODO(jdh): actually getting home from here

    def see_markers(self, predicate=None, attempts=3):
        # type: (Callable[[Marker], bool], int) -> List[Marker]
        """
        Return a list of visible markers that satisfy the given predicate.

        The predicate will be called on each marker, and should return a
        boolean showing whether the marker should be included in the returned
        list. It defaults to None, meaning that all visible markers will be
        returned. Strictly speaking, any falsy markers will be discarded, but
        since markers are instances of collections.namedtuple(), they are
        always truthy.

        If no markers are visible, multiple attempts to see a marker will be
        made, in case of a transient fault with the camera. The number of
        attempts can be changed by altering the attempts parameter, which must
        be greater than zero.
        """
        self.log.info("Looking for markers (%s attempts)...", attempts)
        assert attempts > 0
        markers = []
        for i in xrange(attempts):
            markers = filter(predicate, self.see())
            if markers:
                self.log.info("Found %s markers (attempt %s), returning.", len(markers), i + 1)
                break
            else:
                self.log.debug("No markers found (attempt %s), retrying...", i + 1)
        else:
            self.log.warn("No markers found after %s attempts!", attempts)
        return markers

    def find_closest_marker(self, marker_type):
        # type: (...) -> Marker
        """
        Find and return the closest marker of a given marker type.
        
        If no markers can be found, an IndexError will be raised.
        """
        self.log.info("Finding closest marker of type %s", marker_type)
        markers = [m for m in self.find_markers(filter_func=lambda marker: marker.info.marker_type == marker_type)]
        return sorted(markers, key=attrgetter("dist"))[0]

    def find_markers_approx_position(self, marker_type, dist, dist_tolerance=0.5):
        """
        Find and return a list of markers at an approximate location.

        The list may be empty, in which case no markers could be seen at that distance.
        """
        self.log.info("Finding marker of type %s approximately %s metres away, give or take %s metres", marker_type, dist, dist_tolerance)
        markers = []
        for marker in self.lookForMarkers(max_loop=5):
            if marker.info.marker_type == marker_type and dist - dist_tolerance <= marker.dist <= dist + dist_tolerance:
                self.log.debug("Found a MATCHING %s marker (id %s) %s metres away at %s degrees",
                               marker.info.marker_type, marker.info.code, marker.dist, marker.rot_y)
                markers.append(marker)
            else:
                self.log.debug("Found a non-matching %s marker (id %s) %s metres away at %s degrees",
                               marker.info.marker_type, marker.info.code, marker.dist, marker.rot_y)
        # markers = [m for m in self.lookForMarkers(max_loop=5) if m.info.marker_type == marker_type and dist - dist_tolerance <= m.dist <= dist + dist_tolerance]
        self.log.info("Found %s markers matching criteria", len(markers))
        return markers

    def cone_search(self, marker_type=None, marker_id=None, dist=None,
                    dist_tolerance=0.5, start_angle=-45, stop_angle=45, delta_angle=15):
        # type: (...) -> List[Marker]
        """Search for markers, turning from side to side.

        As soon as markers satisfying the passed criteria are found,
        this function exits and returns them.

        The order of this function's arguments is not stable. Callers
        should use keyword args only, never positional args.
        """
        self.log.info("Starting cone search (%s, %s, %s)...", start_angle, stop_angle, delta_angle)
        self.log.debug("Criteria:")
        self.log.debug("  type == %s", marker_type)
        self.log.debug("  ID == %s", marker_id)
        self.log.debug("  %s <= dist <= %s", dist - dist_tolerance, dist + dist_tolerance)
        angles = [0, start_angle] + [delta_angle for _ in range(0, stop_angle - start_angle, delta_angle)]
        angle_turned = 0

        def predicate(marker):
            # type: (Marker) -> bool
            correct_type = marker_type is None or marker.info.marker_type == marker_type
            correct_id = marker_id is None or marker.info.code == marker_id
            correct_dist = dist is None or dist - dist_tolerance <= marker.dist <= dist + dist_tolerance
            return correct_type and correct_id and correct_dist

        for angle in angles:
            self.wheels.turn(angle)
            angle_turned += angle
            markers = self.see_markers(predicate)
            if markers:
                self.log.info("Found %s markers matching criteria, stopping search.", len(markers))
                return markers
            else:
                time.sleep(0.5)
        self.log.info("Found no markers matching criteria.")
        self.wheels.turn(-angle_turned)  # Turn back to where we were facing originally.
        return []

    def cone_search_approx_position(self, marker_type, dist, dist_tolerance=0.5, max_left=45, max_right=45, delta=15, sleep_time=0.5):
        # type: (...) -> list
        """
        Search for a specific marker type at an appproximate distance with tolerances
        outside of the visual range of the camera
        """
        self.log.info("Doing a cone based search with extremities (%s, %s) and delta %s for markers of type %s approximately %s metres away, give or take %s metres", max_left, max_right, delta, marker_type, dist, dist_tolerance)
        angles = [0, -max_left] + ([delta]*(((max_left + max_right) // delta) + 1))
        for angle in angles:
            self.wheels.turn(angle)
            markers = self.find_markers_approx_position(marker_type, dist, dist_tolerance)
            if markers:
                self.log.info("Finished marker type cone search and found %s markers of type %s", len(markers), marker_type)
                return markers
            time.sleep(sleep_time)
        self.wheels.turn(-max_right)  # close enough
        self.log.info("Finished marker type cone search with no markers found")
        return []

    def cone_search_specific_marker(self, marker_id, max_left=45, max_right=45, delta=15, sleep_time=0.5):
        # type: (...) -> list
        """
        Search for a specific marker outside of the visual range of the camera
        """
        self.log.info("Doing a cone based search with extremities (%s, %s) and delta %s for a marker (id %s)", max_left, max_right, delta, marker_id)
        angles = [0, -max_left] + ([delta]*(((max_left + max_right) // delta) + 1))
        for angle in angles:
            self.wheels.turn(angle)
            markers = self.see_markers(predicate=lambda marker: marker.info.code == marker_id)
            if markers:
                self.log.info("Finished specific marker cone search and found %s markers of id %s", len(markers), marker_id)
                return markers
            time.sleep(sleep_time)
        self.wheels.turn(-max_right)  # close enough
        self.log.info("Finished specific marker cone search with no markers found")
        return []

    def find_markers(self, minimum=1, max_loop=10, delta_angle=20, filter_func=lambda marker: True):
        """
        Find at least minimum markers.
        Try max_loop attempts for each direction.
        """

        # Scan 0.
        self.log.debug("Searching for markers... (direction = 0)")
        markers = self.lookForMarkers(max_loop=max_loop)
        markers = filter(filter_func, markers)
        if len(markers) >= minimum:
            # If found correct number of markers, stop and return them
            return markers
        else:
            angle = delta_angle
            while angle <= 180:
                # If the robot cannot see a marker
                self.log.debug("Searching for markers... (direction = %s)", delta_angle)
                self.wheels.turn(angle)
                markers = filter(filter_func, self.lookForMarkers(max_loop=max_loop))
                if len(markers) >= minimum:
                    self.routeChange = True
                    return markers
                self.wheels.turn(-angle * 2)
                markers = filter(filter_func, self.lookForMarkers(max_loop=max_loop))
                if len(markers) >= minimum:
                    self.routeChange = True
                    return markers
                self.wheels.turn(angle)
                angle += delta_angle
            self.log.error("Couldn't find the requested marker!")
        # Current direction is ~360 (no change)
        self.log.error("Markers (minimum %s) not found with %s loops per direction", minimum, max_loop)
        self.routeChange = True
        return markers 

    def lookForMarkers(self, max_loop=float("inf"), sleep_time=0.5):
        """
        Look for markers.
        if none found within max_loop, return []
        """
        self.log.warn("Deprecation warning: use see_markers instead!")
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

    def get_vec_to_corner(self, marker):
        """
        Given a marker, get the vector to the corner to the left of it.

        See <https://hillsroadrobotics.slack.com/files/anowlcalledjosh/F414B8RPX/office_lens_20170204-144813.jpg>
        or <https://imgur.com/R9E8kpD> for an image showing how this works.
        """
        d = marker.dist
        l = marker.info.offset % 7 + 1
        alpha = marker.rot_y
        beta = marker.orientation.rot_y
        beta_prime = 90 - beta
        self.log.debug("d=%s, l=%s, alpha=%s, beta=%s", d, l, alpha, beta)
        n = sqrt(l**2 + d**2 - 2 * l * d * cosd(beta_prime))
        delta = asind(l * sind(beta_prime) / n)
        self.log.debug("delta=%s", delta)
        gamma = alpha - delta
        return Vector(distance=n, angle=gamma)

    def check_cube_alignment(self):
        """
        Looks to see what cubes are ahead of the robot and checks if they are in the expected position
        """
        RA_marker = False
        LA_marker = False
        C_Marker = False
        self.log.info("Looking for markers to align with")
        markers = self.see_markers()
        for m in markers:
            if m.info.marker_type == MARKER_TOKEN_A:
                if m.rot_y <= 0:
                    RA_marker = self.check_cube_angle(m, -70)
                else:
                    LA_marker = self.check_cube_angle(m, 15)
            if m.info.marker_type == MARKER_TOKEN_B:
                self.check_cube_angle(m, 18)
            if m.info.marker_type == MARKER_TOKEN_C:
                self.log.info("Can see markers with marker type of 'C'")
                if m.rot_y <= -5 or m.rot_y >= 5:
                    self.log.info("'C' marker is out of position, orientation is %s", m.rot_y)
                    C_Marker = False
                else:
                    C_Marker = True
        return [RA_marker, LA_marker, C_Marker]

    def check_cube_angle(self, marker, expectedPosition):
        self.log.info("Can see markers with %s, number %s", marker.info.marker_type, marker.info.code)
        if marker.rot_y <= expectedPosition - 5 or marker.rot_y >= expectedPosition + 5:
            self.log.info("%s marker is out of position, orientation is %s", marker.info.marker_type, marker.rot_y)
            return False
        else:
            return True

    def init_logger(self):
        """
        Initialise logger.
        """
        self.log = logging.getLogger(__name__)
        self.log.setLevel(logging.DEBUG)
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.DEBUG)
        # Example: "filename:42 - do_stuff() - INFO: stuff happened"
        formatter = logging.Formatter("%(module)s:%(lineno)d - %(funcName)s() - %(levelname)s: %(message)s")
        console_handler.setFormatter(formatter)
        self.log.addHandler(console_handler)


if __name__ == "__main__":
    CompanionCube()
