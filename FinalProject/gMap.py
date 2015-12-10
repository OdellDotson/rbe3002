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


class gMap():
    def __init__(self, name):
        self.current_x, self.current_y, self.current_theta = None, None, None
        self._currentSet = False
        self.goalX, self.goalY, self.goaltheta = None, None, None
        self._goalSet = False
        self._map = [[]]
        self._height = 0
        self._width = 0
        self._pose = None
        self._mapSet = False
        self._map_list = tf.TransformListener()
        self._goal_pos = None, None
        self._goal_ = None, None, None
        self.painter = rvptr.rVizPainter(name + " Painter", 0.05)

        print "gMap has been created"

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
        return True

    def getNextFrontier(self, givenMap):
        """
        :return: Returns a pose on the map in Meters for the robot to drive to
        """

        print "Trying to find a new Frontier"

        ## Get the list of frontiers that exist on the map
        frontierList = self.getFrontierList(givenMap)

        print ""
        print "Fontiers found: "
        for n in frontierList:
            print ""
            print n


        print ""


        ## Get the map location in grid cells of the frontier to travel to (can expand to have multiple heuristics for this)
        mapLocationGridCells = self.pickFrontier(frontierList, self.frontierSize)

        ## Return the map locaiton in meters so that the pose can just be gone to
        return self.mapLocationMeters(mapLocationGridCells)

    def getFrontierList(self, givenMap):
        """
        This function finds and creates the list of distinct frontiers and returns it.
        :return: List of Frontiers, each frontier being a list of <(x,y) touples >
        :return: Raise FrontierException (defined in turtleException file) when there are no more frontiers
        """

        result = []

        for y, row in enumerate(givenMap):
            for x, elt in enumerate(row):
                # check to see if a given node is adjacent to the opposite kind of node
                # and check that it isn't in any frontier yet
                isAlreadyFound = False;

                for list in result:
                    if (x, y) in list:
                        isAlreadyFound = True


                if (self.isNodeFrontier(x, y, givenMap) and not isAlreadyFound):
                    frontier = []

                    nodesToExplore = []
                    nodesToExplore.append((x,y))

                    while nodesToExplore.__len__() > 0:
                        currentNode = nodesToExplore.pop()
                        frontier.append(currentNode)

                        for n in self.getNeighborsFrontier(currentNode[0], currentNode[1], givenMap):
                            if not n in frontier and not n in nodesToExplore:
                                nodesToExplore.append(n)

                    result.append(frontier)

        print "Generated Frontier List of ", result.__len__()

        return result

    def frontierSize(self, targetFrontier):
        """
        :param targetFrontier: The frontier, a list of (x,y) points that make up a frontier

        :return: <int> representing the number of 'points' in the frontier
        """
        return len(targetFrontier)
        # TODO: Currently this just picks whichever point in the frontier is closest and goes there. Maybe not the best?

    def pickFrontier(self, frontierList, heuristic):
        """
        :param frontierList: List of Frontiers , list of list of <(x,y) touple > in grid cell location on the map
        :param heuristic: function that only requires a single frontier to make a decision.
        :return:an (x,y) touple of the point to go to in grid cell location on the map
        """
        targetFrontier = frontierList[0]

        print "Trying to find the best frontier"

        if len(frontierList) == 0:  # If there are no frontiers
            raise Exception("Passed in an empty frontier list!")

        ## Select the largest frontier
        elif len(frontierList) != 1:
            for elt in frontierList:
                # Check if the next frontier is a better candidate for travel
                # Assuming a larger Heuristic is better
                if heuristic(targetFrontier) < heuristic(elt):
                    targetFrontier = elt

        print "Found the best frontier, finding the closest point"
        print "Frontier contains ", targetFrontier.__len__(), " nodes"

        ## Find the closest point on the frontier
        currentTarget = targetFrontier[0]
        for elt in targetFrontier:
            # TODO: replace once frontier testing is done
            # if tools.distFormula(elt, (self.current_x, self.current_y)) < tools.distFormula(currentTarget, (
            if tools.distFormula(elt, (0, 0)) < tools.distFormula(currentTarget, (0, 0)):
                currentTarget = elt
                # TODO: Currently this just picks whichever point in the frontier is closest and goes there. Maybe not the best?

        print "Closest point found at: ", currentTarget

        return currentTarget

    def mapLocationMeters(self, mapLocationGridCells):
        """
        :param mapLocationGridCells: (x,y) touple of the location in grid cells
        :return:(x,y) touple of the location in meters
        """
        gridx, gridy = mapLocationGridCells
        mapx = tools.degmapifyValue(gridx)
        mapy = tools.degmapifyValue(gridy)

        return (mapx, mapy)

    def getNeighbors(self, x, y, givenMap, threshold=99):  # TODO Troy has a cleaver way to do this method with try/catches
        """
        This get's the neighbors of a specific point on the map. This function preemptively removes squares with
        values greater than the threshold. This allows us to remove walls and dangerous zones from the path planning

        :param x: x location of the point
        :param y: y location of the point
        :param threshold: threshold to decide if the value is travelable
        :return: None
        """

        height = givenMap[0].__len__()
        width = givenMap.__len__()

        if (y > width or x < 0 or y > height or y < 0):
            return {}
            print (x, y)
            raise ReferenceError("getNeighbors out of bound error on x or y coordinate.")
        # Goes through the values, ignores self
        gen_neighbors = [(x - 1, y - 1),
                         (x + 1, y + 1),
                         (x + 1, y - 1),
                         (x - 1, y + 1),
                         (x, y + 1),
                         (x, y - 1),
                         (x - 1, y),
                         (x + 1, y)]
        goodNeighbors = []
        for move in gen_neighbors:
            tx, ty = move
            try:
                if givenMap[ty][tx] < threshold:
                    goodNeighbors.append(move)
            except IndexError:
                continue
            except Exception, e:
                print "getNeighbors Error:"
                raise e
            """
                NOTE: The above thing is just querying the map and catching the index out of bounds errors if they happen.
                This code may not work if the IndexError is the incorrect error, in this case the 'except IndexError' line would have
                to be fixed. The 'except Exception,e' line is used to make sure that the other exceptions are caught
            """
        return goodNeighbors  # State farm joke goes here

    def getNeighborsFrontier(self, x, y, givenMap):
        """
        Returns all the neighbors of a given node that are frontier nodes
        :param x:
        :param y:
        :return:
        """
        goodNeigbhors = self.getNeighbors(x, y, givenMap)

        edgeNeighbors = []

        for n in goodNeigbhors:
            nodeX, nodeY = n

            if self.isNodeFrontier(nodeX, nodeY, givenMap):
                edgeNeighbors.append(n)

        return edgeNeighbors

    def isNodeFrontier(self, x, y, givenMap):
        """
        Returns true if the given coordinate specific a node that:
            Is known reachable space
            Is adjacent to unknown space
        :param x:
        :param y:
        :return:
        """

        if (givenMap[y][x] == 1):
            neighbors = self.getNeighbors(x, y, givenMap)

            for n in neighbors:
                x, y = n;
                if (givenMap[y][x] == -1):
                    return True

        return False

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
