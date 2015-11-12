__author__ = 'Troy Hughes'

import math


def distFormula(point1, point2):
    """
    :param point1: Touple in the form of (x1, y1)
    :param point2: Touple in the form of (x2, y2)
    :return: Returns Distance between two points
    """
    x1, y1 = point1
    x2, y2 = point2

    return math.sqrt((x1-x2)**2 + (y1-y2)**2)


