from sr.robot import *

import collections
from math import degrees, radians


# All angles should be in radians.
#Vector = collections.namedtuple("Vector", ["distance", "angle"])
class Vector(collections.namedtuple("Vector", ["distance", "angle"])):
    __slots__ = ()
    def __repr__(self):
        return "Vector(distance={}, angle={} degrees)".format(self.distance, degrees(self.angle))


def marker2vector(marker):
    # type: (Marker) -> Vector
    """
    Given a Marker, return a Vector from the camera to the marker.
    """
    return Vector(distance=marker.centre.polar.length, angle=radians(marker.centre.polar.rot_y))
