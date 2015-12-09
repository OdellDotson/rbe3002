__author__ = 'Troy Hughes'

import rospy
import tools
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid

rospy.Publisher('/not_explored_nodes', GridCells, queue_size=1)


class rVizPainter():
    def __init__(self,name, gridSize):
        """
        This funciton takes in the name (for logging purposes) and the size of a grid square. This class
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
        self._painters[painterName] = rospy.Publisher(painterName, GridCells, queue_size=1)
        self._paintNodes[painterName] = None

    def paint(self,painterName, paintList):
        """
        This function is used to publish to Rviz and actually paint the lists.
        it takes a list of (x,y) touples in the passed map scope. This function will convert the
        points to a publishable format and then publish the values to rViz

        :param painterName: <string> Name of the topic to publish to
        :param paintList:   <LIST[(x,y) touple]> that is the points to be painted.
        :return:
        """
        pointList = self._paintListFromTouples(paintList)

        CelltoPublish = GridCells()
        CelltoPublish.header.frame_id = '/map'
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
            x,y = i
            point.x = xfunc(x)
            point.y = yfunc(y)
            point.z = 0
            ret_list.append(point)

        return ret_list


