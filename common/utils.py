from math import *


def next_position_in_circle(x, y, heading, distance):
    est_x = x + distance * cos(heading)
    est_y = y + distance * sin(heading)
    return est_x, est_y


def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
