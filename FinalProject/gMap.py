__author__ = 'Troy Hughes'

import rospy
import math
import tools
import Queue
import tf
import rVizPainter as rvptr
from nav_msgs.msg import GridCells
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from turtleExceptions import FrontierException
from FrontierExplorer import FullMapExplorer


class gMap():
    def __init__(self, name):

        ## Location Information
        self.current_x, self.current_y, self.current_theta = None, None, None
        self._currentSet = False

        ## Map Information
        self._map = [[]]
        self._height = 0
        self._width = 0
        self._pose = None
        self._mapSet = False
        self._map_list = tf.TransformListener()

        #### Contained Classes ####
        ##   Visualiser Class  ##
        self.painter = rvptr.rVizPainter(name + " Painter", 0.05)
        self.painter.addPainter('/testSquares')

        ##   Frontier Explorer ##
        self.FE = FullMapExplorer.FME()



        print "gMap has been created"

    def doneSetup(self):
        if self._mapSet is False:
            return False
        if self.current_x is None or self.current_y is None or self.current_theta is None:
            self._updateLocation()
            return False
        return True

    def updateMap(self, msg):
        """
        This is the function that is called whenever the "/map" topic is published to
        :param msg:
        :return:
        """
        self._updateMap(msg)

    def _updateMap(self, msg):
        self._map = tools.lMaptoLLMap(msg.data,
                                      msg.info.height,
                                      msg.info.width)
        self._max_h = msg.info.height
        self._max_w = msg.info.width
        self._pose = msg.info.origin
        self._mapSet = True
        self._updateLocation()

    def addValue(self, x, y, value):
        if self._mapSet:
            raise RuntimeError("Trying to add with no map")
        try:
            self._map[y][x] = ((self._map[y][x]) * (0.25) + value * (0.75)) / 2.0
        except IndexError:
            pass

    def storeGoal(self, goalX, goalY, goalTheta):
        """
        This stores the goal and get's the current location of the robot in the map.

        :param goalX: x location of the goal in the grid frame
        :param goalY: y location of the goal in the grid frame
        :param goalTheta: angle of the goal in the grid frame (no difference)
        :return: self._goal_, self._current_ which is an : x,y,theta location of the goal and an x,y location of the
            robot.
        """
        self.goalX, self.goalY, self.goaltheta = goalX, goalY, goalTheta
        self._updateLocation()

        print (int(self.goalY), int(self.goalX))

        goal = (self.goalX, self.goalY)
        """ Stores the goal for a* and the goal for xytheta"""
        self._goal_pos = (self.goalX, self.goalY)
        self._goal_ = (self.goalX, self.goalY, self.goaltheta)
        self._updateLocation()  ## Run a second time for map updating assurance
        self._goalSet = True
        return self._goal_, (self.current_x, self.current_y, self.current_theta)

    def _updateLocation(self):
        try:
            self._map_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(0.5))
        except tf.Exception:
            print "Waiting for transform failed"
            self.current_x = tools.gmapifyValue(0)
            self.current_y = tools.gmapifyValue(0)
            self.current_z = tools.gmapifyValue(0)
            return False

        (p, q) = self._map_list.lookupTransform("map", "base_footprint", rospy.Time(0))
        x, y, z = p
        self.current_theta = tools.normalizeTheta(q)

        self.current_x = tools.gmapifyValue(x)
        self.current_y = tools.gmapifyValue(y)
        self.current_z = tools.gmapifyValue(z)
        self._currentSet = True
        self._currentSet = True
        print "Location values are: ",self.current_x,self.current_y,self.current_theta
        return True



    def getNextFrontier(self, Verbose = False):
        """
        :return: Returns a pose on the map in Meters for the robot to drive to
        """

        if Verbose:
            print "Trying to find a new Frontier"

            ## Get the list of frontiers that exist on the map
            frontierList = self.FE.getFrontierList(self._map)
            print ""
            print "Fontiers found: "
            for n in frontierList:
                print ""
                print n
            print ""
        else:
            frontierList = self.FE.getFrontierList(self._map)

        ## Get the map location in grid cells of the frontier to travel to (can expand to have multiple heuristics for this)
        if len(frontierList) == 0:
            print frontierList
            raise FrontierException("The number of frontiers you have are zero")

        ## Pickes the frontier based off the passed heuristic function.
        mapLocationGridCells = self.FE.pickFrontier(frontierList, self.FE.frontierSize)

        ## Return the map locaiton in meters so that the pose can just be gone to
        return self.FE.mapLocationMeters(mapLocationGridCells, self.current_x, self.current_y)



    def isAtGoalPosition(self, currentLoc):
        """
        Takes in a tuple of (x, y) that is the current robot location.
        Returns true if we're there, false otherwise.
        """
        if tools.distFormula(currentLoc,
                             self._goal_pos) < 0.1:  # Maybe this error bar should go out globally, or take it in.
            return True
        else:
            return False

    def isAtGoalAngle(self, currentAngle):
        """
        Takes in the robot's current angle in radians, tests if that angle is the goal angle.
        """
        if abs(currentAngle - self._goal_[2]) < 0.26:  # Like 14 and a half degrees
            return True
        else:
            return False

    def getGoalAngle(self):
        """
        Returns the goal angle of the robot for the turtlebot to keep track of.
        """
        return self._goal_[2]

    def getRobotPosition(self):
        if self.current_x is None:
            print "The positions aren't set", self.current_x, self.current_y
            raise RuntimeError("Positions are still not set!")
        return self.current_x, self.current_y

    def getRobotAngle(self):
        return self.current_theta

    def replaceMap(self, newMap):
        self._map = newMap

    def main(self):
        try:
            pass
        except rospy.ROSInterruptException:
            pass
