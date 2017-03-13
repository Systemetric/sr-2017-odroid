"""A way to store the useful parts of a marker.

This file is part of the code for the Hills Road/Systemetric entry to
the 2017 Student Robotics competition "Easy as ABC".
"""


from sr.robot import *

import collections


class Vector(collections.namedtuple("Vector", ["distance", "angle"])):
    __slots__ = ()

    def __repr__(self):
        return "Vector(distance={}, angle={})".format(self.distance, self.angle)


def marker2vector(marker):
    # type: (Marker) -> Vector
    """
    Given a Marker, return a Vector from the camera to the marker.
    """
    return Vector(distance=marker.centre.polar.length, angle=marker.centre.polar.rot_y)
