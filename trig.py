"""Functions for doing trigonometry in degrees.

This file is part of the code for the Hills Road/Systemetric entry to
the 2017 Student Robotics competition "Easy as ABC".
"""


from math import sin, asin, cos, degrees, radians

def sind(x):
    # type: (float) -> float
    """Return the sine of x degrees."""
    return sin(radians(x))


def cosd(x):
    # type: (float) -> float
    """Return the cosine of x degrees."""
    return cos(radians(x))


def asind(x):
    # type: (float) -> float
    """Return the arc sine of x, in degrees."""
    return degrees(asin(x))
