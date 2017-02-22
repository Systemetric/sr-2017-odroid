from math import sqrt

from trig import sind, cosd, asind
from vector import Vector


# The width/length/height of a cube.
cube_width = 0.245  # +/- 0.010
# The distance from the centre of rotation to the camera.
# webcam_horizontal_offset = 0.245  # Prototype
webcam_horizontal_offset = 0.19  # 2017 robot
# The angle that the camera is angled from the normal by.
# A positive number means the camera is looking to the right.
camera_angular_offset = 0.797903237


def correct_all_cube(vec, beta):
    vec = correct_for_webcam_rotational_placement(vec)
    vec = correct_for_cube_marker_placement(vec, beta)
    vec = correct_for_webcam_horizontal_placement(vec)
    return vec


def correct_for_webcam_horizontal_placement(vec):
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
    m = sqrt(d**2 + r**2 - 2 * d * r * cosd(180 - alpha))
    gamma = asind(d * sind(180 - alpha) / m)
    return Vector(distance=m, angle=gamma)


def correct_for_webcam_rotational_placement(vec):
    # type: (Vector) -> Vector
    """
    Compensate for the crookedness of the camera.

    How this works: imagine the camera is directly facing a cube. `vec.angle`
    is 0, because the robot does not need to turn. Adding a positive number to
    `vec.angle` means that the robot must actually turn right, presumably
    because the camera is looking to the right of the centre-line of the robot.
    """
    return Vector(distance=vec.distance, angle=vec.angle + camera_angular_offset)


def correct_for_cube_marker_placement(vec, beta):
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
    beta = beta
    d = vec.distance
    alpha = vec.angle
    r = cube_width / 2
    m = sqrt(d**2 + r**2 - 2 * d * r * cosd(180 - beta))
    gamma = asind(r * sind(180 - beta) / m)
    return Vector(distance=m, angle=gamma + alpha)
