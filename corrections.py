from math import sin, cos, asin, pi, sqrt, radians

from vector import Vector


cube_width = 0.245  # +/- 0.010
# The distance from the centre of rotation to the webcam.
webcam_horizontal_offset = 0.245  # The robot is ~2 cubes long.

def correct_all_cube(self, vec, beta):
    vec = correct_for_webcam_rotational_placement(vec)
    vec = correct_for_cube_marker_placement(vec, beta)
    vec = correct_for_webcam_horizontal_placement(vec)
    return vec


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
    r = webcam_horizontal_offset
    m = sqrt(d**2 + r**2 - 2 * d * r * cos(pi - alpha))
    gamma = asin(d * sin(pi - alpha) / m)
    return Vector(distance=m, angle=gamma)


def correct_for_webcam_rotational_placement(self, vec):
    # type: (Vector) -> Vector
    """
    Compensate for the crookedness of the camera.
    """
    return Vector(distance=vec.distance, angle=vec.angle - 0.053)


def correct_for_cube_marker_placement(self, vec, beta):
    # type: (Vector, float) -> Vector
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
    beta = radians(beta)
    d = vec.distance
    alpha = vec.angle
    r = cube_width / 2
    m = sqrt(d**2 + r**2 - 2 * d * r * cos(pi - beta))
    gamma = asin(r * sin(pi - beta) / m)
    return Vector(distance=m, angle=gamma + alpha)
