__author__ = 'Troy Hughes'

import rospy
import tools
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from turtleExceptions import PainterException



class rVizPainter():
    def __init__(self,name, gridSize):
        """
        This function takes in the name (for logging purposes) and the size of a grid square. This class
        assumes that all gridsquares that it will be painting to are in fact 'squares' and not 'rectangles'

        :param name: <string> name of the class
        :param gridSize: <float> size of the grid square
        :return:
        """
        self._gridSize = gridSize
        self._name_ = name
        self._painters = {}
        self._paintNodes = {}

    def addPainter(self,painterName):
        """
        This function adds a publisher to the 'painterName' topic, and stores that publisher
        in the class so that it can be referred to later.

        :param painterName: <string> in 'ROS TOPIC' format. This will error if the topic name is not formatted correctly
        :return: None
        """
        if painterName in self._painters:
            raise PainterException("adding this painter will overwrite the painter that already exists, please use distinct naming")

        self._painters[painterName] = rospy.Publisher(painterName, GridCells, queue_size=1)
        self._paintNodes[painterName] = []

    def addPointtoPaint(self,painterName, point):
        """
        Allows the system to store points for painting. All points will be painted by using the 'paint' function.
        :param painterName: <string> this is the topic that the point will get published oni
        :param point: <(x,y) touple> in the map space
        :return:
        """
        if not (painterName in self._painters):
            raise PainterException("The topic you wish to publish to does not exist")

        self._paintNodes[painterName].append(point)

    def paint(self,painterName, paintList=None):
        """
        This function is used to publish to Rviz and actually paint the lists.
        it takes a list of (x,y) touples in the passed map scope. This function will convert the
        points to a publishable format and then publish the values to rViz

        :param painterName: <string> Name of the topic to publish to
        :param paintList:   <LIST[(x,y) touple]> that is the points to be painted.
        :return:
        """

        if not (painterName in self._painters):
            raise PainterException("The painter you wish to paint does not exist")
        if paintList is None: paintList = []

        pointList = self._paintListFromTouples(paintList)

        CelltoPublish = GridCells()
        CelltoPublish.header.frame_id = '/map'
        CelltoPublish.header.stamp = rospy.Time.now()
        CelltoPublish.cell_width = self._gridSize
        CelltoPublish.cell_height = self._gridSize
        CelltoPublish.cells = pointList

        self._painters[painterName].publish(CelltoPublish)

    def _paintListFromTouples(self,toupleList):
        """
        Creates a publishable list of points from a list of x and y touples.

        :param toupleList: <LIST[(x,y) touple]> that is the points to be painted.
        :return: <LIST[points]> that is the points to be painted.
        """
        """
            The below functions (xfunc and yfunc) are the functions that will use
            handle the offsets for rViz
        """
        def xfunc(val):
            return val

        def yfunc(val):
            return val

        ret_list = []
        for i in toupleList:
            point = Point()
            x, y = i
            point.x = xfunc(x)
            point.y = yfunc(y)
            point.z = 0
            ret_list.append(point)

        return ret_list

    def paintGoal(self,painterName, point, dmap=False):
        expandString = ['(x+1,y)','(x-1,y)','(x,y+1)','(x,y-1)']

        toupleList = [point]
        for string in expandString:
            x,y = point
            for i in xrange(20):
                x,y = eval(string)
                toupleList.append((x,y))
        pointList = self._paintListFromTouples(toupleList)

        CelltoPublish = GridCells()
        CelltoPublish.header.frame_id = '/map'
        CelltoPublish.cell_width = self._gridSize
        CelltoPublish.cell_height = self._gridSize
        CelltoPublish.cells = pointList

        self._painters[painterName].publish(CelltoPublish)

        return CelltoPublish
