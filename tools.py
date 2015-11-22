__author__ = 'Troy Hughes'

import math
import tf

from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point

cellW = 0.3
cellH = 0.3
## Created because the cell width and height are the same
cell = 0.3

def distFormula(point1, point2):
    """
    :param point1: Touple in the form of (x1, y1)
    :param point2: Touple in the form of (x2, y2)
    :return: Returns Distance between two points
    """
    # print "DistForm: ",point1, point2

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
    """
    The Point.n values should usually be <Location>*<Size> however in this case
    it is <Location + offsett>*<Size>. This is why the function is so strange. Used
    Guess and check for the offset.
    """
    point.x = (x+(0.3)*7)*0.3
    point.y = (y+(0.3)*1)*0.3
    point.z = z

    return point

def publishListfromTouple(location_list):
    """
    This creates a list of points from a list of tuples

    :param location_list: List of tuples
    :return: list of points
    """
    ret_list = []
    for i in location_list:
        x,y = i
        ret_list.append(makePoint(x,y,0))

    return ret_list


def dialateOccupancyMap(map):
    """
    This function dialates the high value points on a map by 1.

    In other words, if given a map with walls of 100 and open space of 0, it will
    go thorugh and make the walls 1 square bigger in all directions. This is to protect
    the robot.
    :param map:
    :return:
    """
    def dialateNode(map,node,max_x,max_y):
        """
        This dialates one node on a map and returns the map.

        :param map: A list of lists of intigers with min 0 and max 100
        :param node: x,y tuple of the location
        :param max_x: width of the map
        :param max_y: height of the map
        :return:
        """
        x,y = node
        gen_neighbors = [(x-1,y-1),             ## All possible neighbors
                         (x+1,y+1),
                         (x+1,y-1),
                         (x-1,y+1),
                         (x,y+1),
                         (x,y-1),
                         (x-1,y),
                         (x+1,y)]

        for n in gen_neighbors:
            nx,ny = n
            if nx > max_x or ny > max_y or nx < 0 or ny < 0:
                continue
            map[ny][nx] = 100
        return map

    ## Get all the spaces with 100 as their value and put them in a list.
    taken_spaces = []
    for y,row in enumerate(map):
        for x,column in enumerate(row):
            if map[y][x] == 100:
                taken_spaces.append((x,y)) ## (x,y)
    max_y = y
    max_x = x
    ## Dialate all the spaces in the list.
    for space in taken_spaces:
        map = dialateNode(map,space,max_x,max_y)

    return map

def lMaptoLLMap(lMap, height, width):
    """
    Converts a List Map to a List of Lists map.

    In ROS, many maps are of the form [ROW1,ROS2,ROW3...ROSn]. This simplifies them
    to [[ROW1],[ROW2],...[ROWn]]. This allows you to index them as: point = map[y][x] to get
    the value at a specific map.

    :param lMap:
    :param height:
    :param width:
    :return:
    """
    map = []
    for i in xrange(height):
        new_list = []
        new_list.extend(lMap[i*width:(i+1)*width])
        map.append(new_list)
    return map

def mapifyValue(value):
    return int(round(value/cell))


def demapifyValue(value):
    return value*cell


