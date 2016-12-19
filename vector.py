from sr.robot import *

import collections


# All angles should be in radians.
Vector = collections.namedtuple("Vector", ["distance", "angle"])


def marker2vector(marker):
    # type: (Marker) -> Vector
    """
    Given a Marker, return a Vector from the camera to the marker.
    """
    return Vector(distance=marker.centre.polar.length, angle=marker.centre.polar.rot_y)