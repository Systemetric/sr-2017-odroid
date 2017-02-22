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
