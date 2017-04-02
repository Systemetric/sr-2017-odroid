"""A collection of strategies the robot should use to collect cubes.

This file is part of the code for the Hills Road/Systemetric entry to
the 2017 Student Robotics competition "Easy as ABC".
"""


from __future__ import division

from sr.robot import *

from collections import Callable, Hashable
from math import sqrt
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
def route_b_c_a(robot, opposite_direction=False, skip_initial_walk=False):
    if opposite_direction:
        turn_factor = -1
    else:
        turn_factor = 1
    hasB = False
    if not skip_initial_walk:
        robot.log.debug("Moving 3.25 metres to next to B")
        initial_walk_successful = robot.move_continue(3.25)
        robot.wheels.turn(-90 * turn_factor)
        time.sleep(0.2)
    else:
        robot.log.info("Skipping initial walk, presumably someone put the wrong USB stick in...")
        initial_walk_successful = True
    if initial_walk_successful:
        markers = robot.find_markers_approx_position(MARKER_TOKEN_B, 1.5)
    else:
        robot.log.warn("Initial walk had a problem, doing a cone search since we don't know where we are.")
        if opposite_direction:
            angles = {"start_angle"}
        else:
            angles = {}
        markers = robot.cone_search(marker_type=MARKER_TOKEN_B, dist=1.5, dist_tolerance=1, **angles)
    if markers:
        robot.log.debug("Found %s B cubes, moving to the 0th one (code %s)", len(markers), markers[0].info.code)
        marker = markers[0]
        hasB = True
        validMovement = robot.move_to_cube(marker)
        if validMovement == 'Crash':
            robot.log.debug("Moving 1.0 metres backwards to get a better view of B because of a collision")
            time.sleep(1)
            robot.move_continue(-1)
            hasB = False
            robot.log.debug("Trying to find B again")
            markers = robot.find_markers_approx_position(MARKER_TOKEN_B, 1.5)
            if markers == []:
                robot.log.warn("Can't see B cube!")
                position_markers = robot.check_cube_alignment()
                if position_markers[2] == False:
                    robot.log.warn("Cannot see B or C cube, in the wrong place")
            else:
                robot.log.debug("Found %s B cubes, moving to the 0th one", len(markers))
                marker = markers[0]
                hasB = True
                validMovement = robot.move_to_cube(marker)
                if validMovement == 'Crash':
                    robot.log.debug("Moving 0.2 metres backwards to unhook from a collision")
                    time.sleep(1)
                    robot.wheels.move(-0.2, ignore_crash=True)
                    hasB = False

    if hasB == False:
        robot.log.info("Having not found the first B cube, finding C cube")
        Cmarkers = robot.find_markers_approx_position(MARKER_TOKEN_C, 2.88)
        if Cmarkers == []:
            robot.log.warn("Can't see C cube!")
            # Idiot check -- did we turn the wrong way?
            robot.log.debug("Checking to see if someone put the wrong USB stick in...")
            robot.wheels.turn(180)
            markers = robot.cone_search(marker_type=MARKER_TOKEN_B, dist=1.5, start_angle=-30, stop_angle=30)
            if markers:
                robot.log.warn("Someone put the wrong USB stick in the robot! Starting again in the other direction.")
                route_b_c_a(robot, opposite_direction=not opposite_direction, skip_initial_walk=True)
                robot.log.info("Done recursing, we're probably home now.")
                return
            else:
                robot.log.info("We can't see a B cube behind us, presumably all the other robots are just really good.")
                robot.wheels.turn(180)  # Turn back to face the original direction.
            robot.log.debug("Turning to roughly A cube")
            robot.wheels.turn(-45 * turn_factor)
            Amarkers = robot.cone_search_approx_position(MARKER_TOKEN_A, dist=1.8, dist_tolerance = 0.7)
            if Amarkers:
                marker_id = Amarkers[0].info.code
                robot.log.info("Having not found B nor C cube, and found an A cube, turning to face A cube (%s) exactly", marker_id)
                robot.face_cube(Amarkers[0])
                robot.log.info("Moving to A cube (not from a corner)")
                robot.wheels.turn(-45 * turn_factor)
                # a^2 = 2b^2 (right-angled isoceles triangle, hypotenuse a)
                # a^2 / 2 = b^2
                # sqrt(a^2 / 2) = b
                robot.move_continue(sqrt(Amarkers[0].dist**2 / 2))
                robot.wheels.turn(90 * turn_factor)
                Amarkers = robot.cone_search_specific_marker(marker_id, max_left=30, max_right=30)
                if Amarkers:
                    marker = Amarkers[0]
                    robot.move_to_cube(marker, crash_continue=True)
                    Bmarkers = robot.cone_search_approx_position(MARKER_TOKEN_B, dist=1.0, max_left=30, max_right=30)
                    if Bmarkers:
                        robot.log.debug("Having not found B nor C cube but getting A cube, found a B cube, going for it")
                        marker = Bmarkers[0]
                        robot.move_to_cube(marker, crash_continue=True)
                        robot.log.debug("Now going home")
                        robot.wheels.turn(180)
                        robot.move_continue(1.5)
                        robot.wheels.turn(45 * turn_factor)
                        robot.move_home_from_A()
                        robot.log.info("Home?")
                    else:
                        robot.log.debug("Having not found B nor C cube but getting A cube, second B not visible, going home")
                        robot.wheels.turn(-135 * turn_factor)
                        robot.move_home_from_A()
                        robot.log.info("Home?")
                else:
                    robot.log.fatal("Having not found B nor C cube, found an A cube but it has moved when we weren't looking!")
            else:
                robot.log.fatal("Can't find any cubes at all where I expect them to be!")
        else:
            robot.log.debug("Having NOT found a B cube, found %s C cubes, moving to the 0th one", len(Cmarkers))
            marker = Cmarkers[0]
            robot.log.info("Moving to C cube")
            # TODO: Add collsions stuff here, raises exception on failure
            robot.move_to_cube(marker, crash_continue=True)
            robot.log.debug("Hasn't got B cube so turning to roughly B cube")
            robot.wheels.turn(-90 * turn_factor)
            Bmarkers = robot.find_markers_approx_position(MARKER_TOKEN_B, 1.5)
            if Bmarkers == []:
                robot.log.debug("Cannot see a B, turning to roughly A cube")
                robot.wheels.turn(-45 * turn_factor)
                markers = robot.cone_search_approx_position(MARKER_TOKEN_A, 2.12, max_left=30, max_right=30)
            else:
                robot.log.debug("Found %s B cubes, moving to the 0th one", len(Bmarkers))
                marker = Bmarkers[0]
                robot.move_to_cube(marker, crash_continue=True)
                robot.log.debug("turning to roughly A cube")
                robot.wheels.turn(-90 * turn_factor)
                markers = robot.cone_search_approx_position(MARKER_TOKEN_A, 1.5, max_left=30, max_right=30)
            robot.log.info("Moving to A cube")
            if markers:
                marker = markers[0]
                robot.move_to_cube(marker, crash_continue=True)
                if Bmarkers:
                    robot.log.debug("Turning to face home")
                    robot.wheels.turn(45 * turn_factor)
            else:
                robot.log.info("Can't see A cube, moving to roughly where it should be.")
                if Bmarkers == []:
                    robot.wheels.move(2.12)  # Move from C cube
                else:
                    robot.wheels.move(1.5)  # Move from B cube
                    robot.wheels.turn(45 * turn_factor)
            robot.move_home_from_A()
            robot.log.info("Home?")
    else:
        if marker.info.marker_type != MARKER_TOKEN_B:
            robot.log.warn("Last marker we saw wasn't a B marker despite being at a B cube, this is very weird.")
        else:
            robot.log.debug("Correcting for turn we made to face B cube (%s degrees)", marker.rot_y)
            robot.wheels.turn(-marker.rot_y)
        robot.log.info("We have a B cube! Finding C cube")
        Cmarkers = robot.find_markers_approx_position(MARKER_TOKEN_C, 1.0)
        if Cmarkers == []:
            robot.log.warn("Can't see C cube!")
            robot.log.debug("Cannot see a C, turning to roughly A cube")
            robot.wheels.turn(-90 * turn_factor)
            Amarkers = robot.find_markers_approx_position(MARKER_TOKEN_A, 1.0)
            if Amarkers:
                marker = Amarkers[0]
                robot.log.info("Moving to A cube")
                robot.move_to_cube(marker, crash_continue=True)
                robot.log.debug("going home")
                robot.wheels.turn(-45 * turn_factor)
                robot.move_home_from_A()
                robot.log.info("Home?")
            else:
                # This is pretty soon after the round starts, so it's odd that our A cube is not
                # visible.
                robot.log.info("Can't see A cube, going home with just a B cube")
                # TODO(jdh): check if arena markers behind A cube are visible. For now, assume that
                # they are, so go home via the missing A cube.
                robot.move_continue(1.5)
                robot.wheels.turn(-45 * turn_factor)
                robot.move_home_from_A()
                robot.log.info("Home?")
        else:
            robot.log.debug("Found %s C cubes, moving to the 0th one", len(Cmarkers))
            marker = Cmarkers[0]
            robot.log.info("Moving to C cube")
            validMovement = robot.move_to_cube(marker)
            if validMovement == 'Crash':
                robot.log.debug("Moving 0.5 metres backwards to get a better view of C because of a collision")
                time.sleep(1)
                robot.wheels.move(-0.5, ignore_crash=True)
                robot.log.debug("Trying to find C again")
                markers = robot.find_markers_approx_position(MARKER_TOKEN_C, 1.5, 2)
                if markers == []:
                    robot.log.warn("Cannot see C cube, attempting to get an A cube")
                    robot.wheels.turn(-117 * turn_factor)
                else:
                    validMovement = robot.move_to_cube(markers[0])
                    if validMovement == 'Crash':
                        robot.log.debug("Moving 0.2 metres backwards to unhook from a collision")
                        time.sleep(1)
                        robot.wheels.move(-0.2, ignore_crash=True)
                        robot.log.warn("Cannot see C cube, attempting to get an A cube")
                        robot.wheels.turn(-117 * turn_factor)
                    else:
                        robot.log.info("Got B, got C and searching for A")
                        robot.wheels.turn(-135 * turn_factor)
                markers = robot.cone_search_approx_position(MARKER_TOKEN_A, dist=1.3)
                if markers:
                    marker = markers[0]
                    robot.move_to_cube(marker, crash_continue=True)
                    robot.log.info("Got B, lost C and got A cube.")
                else:
                    robot.log.info("Can't see A cube, going to roughly where it should be.")
                    robot.move_continue(2.12)
                robot.log.debug("Going home...")
                robot.move_home_from_A()
            else:
                robot.log.debug("Has B and C so turning to roughly A cube")
                robot.wheels.turn(-135 * turn_factor)
                markers = robot.cone_search(marker_type=MARKER_TOKEN_A, dist=2.12, start_angle=-30, stop_angle=30)
                if markers:
                    marker = markers[0]
                    robot.log.info("Moving to A cube")
                    robot.move_to_cube(marker, crash_continue=True)
                    robot.log.debug("Now at A cube")
                else:
                    robot.log.info("Can't see A cube, going to roughly where it should be.")
                    robot.move_continue(2.12)
                time.sleep(1)
                robot.move_home_from_A()
                robot.log.info("Home?")


def route_c_b_a(robot):
    robot.log.info("Finding C cube")
    Cmarkers = robot.find_markers_approx_position(MARKER_TOKEN_C, 3.25)
    if Cmarkers == []:
        # TODO(jdh): do something sane when cubes are missing
        robot.log.warn("Can't see C cube!")
        position_markers = robot.check_cube_alignment()
        if position_markers[2] == False:
            robot.log.warn("Cannot see B or C cube, in the wrong place")
        else:
            # Change strategy
            robot.log.debug("Cannot see a C, turning to roughly A cube")
            robot.wheels.turn(-45)
            marker = robot.find_closest_marker(MARKER_TOKEN_A)
            robot.log.info("Moving to A cube")
            robot.move_to_cube(marker)
            robot.log.debug("going home")
            robot.wheels.turn(-135)
            robot.wheels.move(2)
    else:
        robot.log.debug("Found %s C cubes, moving to the 0th one", len(Cmarkers))
        marker = Cmarkers[0]
        robot.log.info("Moving to C cube")
        robot.move_to_cube(marker)
    robot.log.debug("Hasn't got B cube so turning to roughly B cube")
    robot.wheels.turn(-90)
    Bmarkers = robot.find_markers_approx_position(MARKER_TOKEN_B, 1.5)
    if Bmarkers == []:
        robot.log.debug("Cannot see a B, turning to roughly A cube")
        robot.wheels.turn(-45)
        marker = robot.find_closest_marker(MARKER_TOKEN_A)
        robot.log.info("Moving to A cube")
        robot.move_to_cube(marker)
        robot.log.debug("going home")
        robot.wheels.move(2)
    else:
        robot.log.debug("Found %s B cubes, moving to the 0th one", len(Bmarkers))
        marker = Bmarkers[0]
        robot.move_to_cube(marker)
        robot.log.debug("turning to roughly A cube")
        robot.wheels.turn(-45)
        marker = robot.find_closest_marker(MARKER_TOKEN_A)
        robot.log.info("Moving to A cube")
        robot.move_to_cube(marker)
        robot.log.debug("turning to face home")
        robot.wheels.turn(45)
        robot.wheels.move(2)


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


@strategy("align cubes")
def test_align_markers(robot):
    robot.check_cube_alignment()


@strategy("print all cubes in sight")
def test_marker_print(robot):
    markers = robot.see_markers()
    robot.log.info(markers)
    

@strategy("test move 4 metres")
def move_4_metres(robot):
    robot.wheels.move(2)
    robot.wheels.move(2)


@strategy("test 4 metre square")
def square_4_metres(robot, opposite_direction=False):
    if opposite_direction:
        turn_factor = -1
    else:
        turn_factor = 1
    robot.log.debug("opposite_direction is %s", opposite_direction)
    for _ in xrange(4):
        robot.log.debug("Moving forwards")
        robot.move_continue(4)
        robot.log.debug("Turning %s degrees", -90 * turn_factor)
        robot.wheels.turn(-90 * turn_factor)


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
def test_webcam_rotational_placement_correction_calibration(robot, *args, **kwargs):
    while True:
        robot.wheels.move(-2)
        markers = robot.lookForMarkers()
        for marker in markers:
            print "marker type = %s" % (marker.info.marker_type)
            print "marker.rot_y = %s" % (marker.rot_y)
            print "corrected marker.rot_y = %s" % (corrections.correct_for_webcam_rotational_placement(marker2vector(marker)).angle)
        time.sleep(2)
        robot.wheels.move(2)
        time.sleep(10)
        print "##### Restarting... #####"


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


@strategy("test_marker_drive_home")
def test_marker_drive_home(robot):
    arena_marker = robot.find_closest_marker(MARKER_ARENA)
    robot.log.info("Marker(type=%s, id=%s, distance=%s, angle=%s)",
                arena_marker.info.marker_type,
                arena_marker.info.code,
                arena_marker.dist,
                arena_marker.rot_y)
    vec = robot.get_vec_to_corner(arena_marker)
    robot.log.info("Corner: %r", vec)
    robot.wheels.turn(vec.angle)
    robot.wheels.move(vec.distance)


@strategy("test marker attributes")
def test_marker_attributes(robot, *args, **kwargs):
    for _ in xrange(4):
        robot.log.debug(robot.see_markers())
        time.sleep(1)


@strategy("test are we moving")
def test_are_we_moving(robot, *args, **kwargs):
    a = robot.see_markers()
    time.sleep(1)
    b = robot.see_markers()
    robot.log.debug("We should not have moved -- we should log False now.")
    robot.log.debug(robot.are_we_moving(a, b))
    a = robot.see_markers()
    robot.wheels.move(0.5)
    b = robot.see_markers()
    robot.log.debug("We should have moved -- we should log True now.")
    robot.log.debug(robot.are_we_moving(a, b))


@strategy("test going home from anywhere")
def test_going_home(robot, *args, **kwargs):
    robot.move_home_from_other_A()


@strategy("print visible codes forever")
def print_visible_codes_forever(robot, *args, **kwargs):
    while True:
        robot.log.debug("Looking for markers...")
        markers = robot.see_markers()
        if markers:
            for marker in markers:
                robot.log.debug("  I can see marker %s, %s metres away.", marker.info.code, marker.dist)
        else:
            robot.log.debug("  I can't see any markers :(")
        time.sleep(2)
