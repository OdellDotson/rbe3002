__author__ = 'Troy Hughes'

import math
import tf

from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point


def distFormula(point1, point2):
    """
    :param point1: Touple in the form of (x1, y1)
    :param point2: Touple in the form of (x2, y2)
    :return: Returns Distance between two points
    """
    x1, y1 = point1
    x2, y2 = point2

    return math.sqrt((x1-x2)**2 + (y1-y2)**2)


def normalizeTheta(quaternian_touple):
    """
    This takes in a quaternian touple and returns a 0 to 2Pi value for theta

    :param quaternian_touple: takes in the (x,y,z,w) touple
    :return: a normalized theta from this function
    """
    euler_angles = tf.transformations.euler_from_quaternion(quaternian_touple)
    ## This will create a euler angle list from the quaternian information
    ## it will be in order of [Roll, Pitch, Yaw] >> Yaw is the rotation about the
    ##     ## z axis where the robot is driving in the xy plane.
    un_normalized_theta = euler_angles[2] ## This theta goes from [0,pi,-pi,0] where [0:0, pi:179 degrees, -pi:181 degrees]

    # Fixes the [0,pi,-pi,0] problem, translates to [0,2pi] over 360
    if un_normalized_theta > 0:normalized_theta = un_normalized_theta
    else: normalized_theta = (math.pi + (un_normalized_theta)) + math.pi

    return normalized_theta

def makeGridCells(name, width, height, grid_cells=[]):
    """

    :param name: of the topic on which the grid cells are from
    :param width: of a grid cell
    :param height: of a grid cell
    :return: the created grid cell
    """
    cells = GridCells()
    cells.header.frame_id = name
    cells.cell_width = width
    cells.cell_height = height
    cells.cells = grid_cells

    return cells


def makePoint(x,y,z=0):
    point = Point()
    point.x = x
    point.y = y
    point.z = z

    return point