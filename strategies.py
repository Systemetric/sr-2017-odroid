"""A collection of strategies the robot should use to collect cubes."""


from sr.robot import *

from collections import Callable, Hashable
from math import degrees, radians
import time

import corrections
from vector import marker2vector

strategies = {}


def strategy(name):
    # type: (Hashable) -> Callable
    """Register a strategy so that it can be accessed later."""
    # http://thecodeship.com/patterns/guide-to-python-function-decorators/
    # Usage:
    #     @strategy("a friendly strategy name")
    #     def route_nine():
    #         ...
    #
    # Then, in a subclass of Robot:
    #     strategies["a friendly strategy name"](self)
    def wrap(fn):
        strategies[name] = fn
        return fn
    return wrap


@strategy("b c a")
def route_b_c_a(robot):
    robot.log.debug("Moving 3.75 metres to next to B")
    robot.wheels.move(3.25)
    robot.wheels.turn(-90)
    time.sleep(0.2)
    # markers = robot.find_marker_approx_position(MARKER_TOKEN_B, 1.5)
    # if markers == []:
    #     # TODO(jdh): do something sane when cubes are missing
    #     robot.log.warn("Can't see B cube! Stopping strategy.")
    #     return
    # else:
    # robot.log.debug("Found %s B cubes, moving to the 0th one", len(markers))
    # marker = markers[0]
    # FIXME
    marker = robot.find_markers(filter_func=lambda markers: [m for m in markers if m.info.marker_type == MARKER_TOKEN_B and 1.5 - 0.25 <= m.dist <= 1.5 + 0.25])[0]
    robot.log.info("Moving to B cube")
    robot.move_to_cube(marker)
    time.sleep(0.2)
    robot.log.info("Finding C cube")
    marker = robot.find_closest_marker(MARKER_TOKEN_C)
    robot.log.info("Moving to C cube")
    robot.move_to_cube(marker)
    robot.log.debug("turning to roughly A cube")
    robot.wheels.turn(-135)
    time.sleep(0.2)
    robot.log.info("Finding A cube")
    marker = robot.find_closest_marker(MARKER_TOKEN_A)
    robot.log.info("Moving to A cube")
    robot.move_to_cube(marker, distance_after=2.5)


@strategy("a c b preinit")
def point_to_b_preinit(robot):
    marker = robot.find_closest_marker(MARKER_TOKEN_A)
    robot.log.info("Facing towards A cube")
    distance = robot.face_cube(marker)
    return [distance]


@strategy("a c b")
def route_a_c_b(robot, initial_distance):
    robot.log.info("Moving to A cube")
    robot.wheels.move(initial_distance)
    marker = robot.find_closest_marker(MARKER_TOKEN_C)
    robot.log.info("Moving to C cube")
    robot.move_to_cube(marker, max_safe_distance=3)
    robot.wheels.turn(-135)
    robot.log.info("Finding B cube")
    markers = robot.find_specific_markers(MARKER_TOKEN_B, delta_angle=90)
    while not markers:
        robot.log.error("Could not find ANY B cubes!")
        markers = robot.find_specific_markers(MARKER_TOKEN_B, delta_angle=90)
    marker = markers[0]
    robot.log.info("Moving to B cube")
    robot.move_to_cube(marker)
    robot.log.error("NOT IMPLEMENTED - GO HOME")


@strategy("test move 4 metres")
def move_4_metres(robot):
    robot.wheels.move(2)
    robot.wheels.move(2)


@strategy("test move forward")
def test_move(robot):
    robot.wheels.move(0.5)


@strategy("test turn 10 times")
def turn_10_times(robot):
    for i in xrange(20):
        robot.wheels.turn(180)


@strategy("test turn once")
def turn_once(robot):
    robot.wheels.turn(180)
    robot.wheels.turn(180)


@strategy("test webcam rotational placement correction calibration")
def test_webcam_rotational_placement_correction_calibration(robot):
    robot.wheels.move(-2)
    markers = robot.lookForMarkers()
    for marker in markers:
        print "marker type = %s" % (marker.info.marker_type)
        print "marker.rot_y = %s (deg)" % (marker.rot_y)
        print "corrected marker.rot_y = %s (deg)" % (degrees(corrections.correct_for_webcam_rotational_placement(marker2vector(marker)).angle))


@strategy("test cube marker placement correction")
def route_test_cube_marker_placement_correction(robot):
    while True:
        print "----------"
        markers = robot.lookForMarkers()
        for marker in markers:
            vec = marker2vector(marker)
            vec = robot.correct_for_webcam_rotational_placement(vec)
            print "original vector:", vec
            print "      corrected:", robot.correct_for_cube_marker_placement(vec, marker.orientation.rot_y)
        time.sleep(5)


@strategy("test all corrections")
def route_test_all_corrections(robot):
    while True:
        print "----------"
        markers = robot.lookForMarkers()
        for marker in markers:
            vec = marker2vector(marker)
            print "original vector:", vec
            vec = robot.correct_for_cube_marker_placement(vec, marker.orientation.rot_y)
            vec = robot.correct_for_webcam_horizontal_placement(vec)
            print "      corrected:", vec
        time.sleep(5)


@strategy("test moving")
def route_test_moving(robot):
    markers = robot.lookForMarkers()
    for marker in markers:
        vec = marker2vector(marker)
        print "original vector:", vec
        print "      corrected:", robot.correct_for_cube_marker_placement(vec, marker.orientation.rot_y)
    robot.wheels.move(2.5)
    robot.wheels.move(1)
    markers = robot.lookForMarkers()
    while True:
        for marker in markers:
            vec = marker2vector(marker)
            print "original vector:", vec
            print "      corrected:", robot.correct_for_cube_marker_placement(vec, marker.orientation.rot_y)
        time.sleep(3)


@strategy("test")
def route_test(robot):
    """
    Move onto marker A.
    """
    robot.log.debug("Starting route")
    robot.log.debug("Facing marker")
    robot.wheels.turn(45)  # Turn ~45 degrees to face the marker
    marker = robot.find_markers()[0]
    vec = marker2vector(marker)
    vec = robot.correct_for_cube_marker_placement(vec, marker.orientation.rot_y)
    vec = robot.correct_for_webcam_horizontal_placement(vec)
    robot.log.debug("Fine-tuning turn by %s degrees", degrees(vec.angle))
    robot.wheels.turn(degrees(vec.angle))
    #robot.faceMarker(marker)  # Perform corrections to face the marker
    robot.log.debug("Moving to cube")
    robot.moveToCube()
    robot.log.debug("On top of cube")
    robot.wheels.move(0.1)  # Travel past the cube
    robot.log.debug("Finished route")


@strategy("print vectors")
def route_test_vector(robot):
    while True:
        robot.log.debug("finding markers...")
        markers = robot.lookForMarkers(max_loop=3)
        robot.log.debug("found %s markers", len(markers))
        for marker in markers:
            vec = marker2vector(marker)
            robot.log.debug("found marker with vector %s", vec)
            robot.log.debug("vector to centre of cube: %s", robot.correct_for_cube_marker_placement(vec, marker.orientation.rot_y))
        time.sleep(5)


@strategy("test_marker_id_types")
def test_marker_id_types(robot):
    while True:
        markers = robot.lookForMarkers()
        for marker in markers:
            vec = marker2vector(marker)
            robot.log.info("Marker(type=%s, id=%s, distance=%s, angle=%s)",
                marker.info.marker_type,
                marker.info.code,
                vec.distance,
                degrees(vec.angle))


@strategy("test_marker_drive_home")
def test_marker_drive_home(robot):
    arena_marker = robot.find_closest_marker(MARKER_ARENA)
    robot.log.info("Marker(type=%s, id=%s, distance=%s, angle=%s (deg))",
                arena_marker.info.marker_type,
                arena_marker.info.code,
                arena_marker.dist,
                arena_marker.rot_y)
    vec = robot.get_vec_to_corner(arena_marker)
    robot.log.info("Corner: %r", vec)
    robot.wheels.turn(degrees(vec.angle))
    robot.wheels.move(vec.distance)
