__author__ = 'Troy Hughes'


import rospy
import math
import tools
import Queue
import tf
from nav_msgs.msg import GridCells
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point


class gMap():
    def __init__(self, name):
        self.current_x, self.current_y, self.current_theta = None, None, None
        self._currentSet = False
        self.goalX,self.goalY,self.goaltheta = None, None, None
        self._goalSet = False
        self._map = [[]]
        self._height = 0
        self._width = 0
        self._pose = None
        self._mapSet = False
        self._map_list = tf.TransformListener()
        self._goal_pos = None, None
        self._goal_ = None, None, None

        print "gMap has been created"


    def updateMap(self,msg):
        self._updateMap(msg)


    def _updateMap(self,msg):
        print "_updateMap has been called"
        self._map = tools.lMaptoLLMap(msg.data,
                                      msg.info.height,
                                      msg.info.width)
        self._max_h = msg.info.height
        self._max_w = msg.info.width
        self._pose = msg.info.origin
        self._mapSet = True
        self._updateLocation()


    def addValue(self, x,y,value):
        if self._mapSet:
            raise RuntimeError("Trying to add with no map")
        try:
            self._map[y][x] = ((self._map[y][x])*(0.25) + value*(0.75))/2.0
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
        self.goalX,self.goalY,self.goaltheta = goalX,goalY,goalTheta
        self._updateLocation()

        print (int(self.goalY), int(self.goalX))

        goal = (self.goalX, self.goalY)
        """ Stores the goal for a* and the goal for xytheta"""
        self._goal_pos = (self.goalX, self.goalY)
        self._goal_ = (self.goalX, self.goalY, self.goaltheta)
        self._updateLocation() ## Run a second time for map updating assurance
        self._goalSet = True
        return self._goal_, (self.current_x,self.current_y,self.current_theta)


    def _updateLocation(self):
        try:
            self._map_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(0.5))
        except tf.Exception:
            print "Waiting for transform failed"
            self.current_x = tools.gmapifyValue(0)
            self.current_y = tools.gmapifyValue(0)
            self.current_z = tools.gmapifyValue(0)
            return False

        (p,q) = self._map_list.lookupTransform("map","base_footprint",rospy.Time(0))
        x,y,z = p
        self.current_theta = tools.normalizeTheta(q)

        self.current_x = tools.gmapifyValue(x)
        self.current_y = tools.gmapifyValue(y)
        self.current_z = tools.gmapifyValue(z)
        self._currentSet = True
        self._currentSet = True
        return True

    def getNextFrontier(self):
       """
       :return: Returns a pose on the map in Meters for the robot to drive to
       """
       frontierList = self.getFrontierList()                            ## Get the list of frontiers that exist on the map
       mapLocationGridCells = self.pickFrontier(frontierList)           ## Get the map location in grid cells of the frontier to travel to (can expand to have multiple heuristics for this)

       return self.mapLocationMeters(mapLocationGridCells)              ## Return the map locaiton in meters so that the pose can just be gone to

    def getFrontierList(self):
       """
       This function finds and creates the list of distinct frontiers and returns it.
       :return: List of Frontiers, each frontier being a list of <(x,y) touples >
       """
       result = [[]]

       for y,row in enumerate(self._map):
           for x,elt in enumerate(row):
                # check to see if a given node is adjacent to the opposite kind of node
                # and check that it isn't in any frontier yet
                if (self.isFrontierNode(elt) and not elt in result):
                    frontier = []

                    nodesToExplore = Queue.Queue()
                    nodesToExplore.put(elt)


                    while not nodesToExplore.empty():
                        frontier.append(nodesToExplore.pop())


                # create new frontier (list of nodes) and add this node to it
                # create queue of frontiers to explore


                # check neighbors: is reachable space, adjacent to an unknown space, isn't in the frontier yet
                    # add to frontier
                    # add to queue to explore
                # repeat for next node in the queue until the queue is empty

                # once the frontier is complete add it to the frontier list

                # this gets repeated until all the map has been explored




       raise NotImplementedError("getFrontierList not implemented yet")

    def isFrontierNode(self, node):
        """
        This returns true if the node provided is both
            reachable known space
            has a neighbor that is unknown space
        :param node:
        :return:
        """
        result = False;

        neighbors = self.getNeighbors(node)

        for elt in neighbors:
            x, y = elt
            result |= self._map[y][x] == 1

        return result


    def frontierHeuristic(self, targetFrontier):
        """
        :param targetFrontier: The frontier, a list of

        :return: a value that corresponds to that specific frontier's heuristic value
        """
        pass

    def pickFrontier(self, frontierList):
       """

       :param frontierList: List of Frontiers , list of list of <(x,y) touple > in grid cell location on the map
       :return:an (x,y) touple of the point to go to in grid cell location on the map
       """

       targetFrontier = frontierList[0];

       if frontierList.len() == 0:#If there are no frontiers
           raise Exception("Passed in an empty frontier list!")

       elif frontierList.len() != 1:#If we're given more than a single frontier
           for elt in frontierList:
               if frontierHeuristic(targetFrontier) < frontierHeuristic(elt):#Check if the next frontier is a better candidate for travel
                   targetFrontier = elt;

       for elt in targetFrontier:
            return targetFrontier[int(targetFrontier.len()/2)]


       #Do the thing to targetFrontier to get the point

       raise NotImplementedError("pickFrontier not implemented yet")

    def mapLocationMeters(self, mapLocationGridCells):
       """
       :param mapLocationGridCells: (x,y) touple of the location in grid cells
       :return:(x,y) touple of the location in meters
       """
       raise NotImplementedError("mapLocationMeters not implemented yet")



    def getNeighbors(self,x,y,threshold=99): #TODO Troy has a cleaver way to do this method with try/catches
        """
        This get's the neighbors of a specific point on the map. This function preemptively removes squares with
        values greater than the threshold. This allows us to remove walls and dangerous zones from the path planning

        :param x: x location of the point
        :param y: y location of the point
        :param threshold: threshold to decide if the value is travelable
        :return: None
        """

        if(y>self._width or x < 0 or y>self._height or y < 0):
            print (x,y)
            raise ReferenceError("getNeighbors out of bound error on x or y coordinate.")
        #Goes through the values, ignores self
        gen_neighbors = [(x-1,y-1),
                         (x+1,y+1),
                         (x+1,y-1),
                         (x-1,y+1),
                         (x,y+1),
                         (x,y-1),
                         (x-1,y),
                         (x+1,y)]
        goodNeighbors = []
        for move in gen_neighbors:
            tx, ty = move
            if ((tx == x) and (ty==y)) or (tx > self._width or x < 0 or y > self._height or y < 0):
                continue
            if(self._map[ty][tx] < threshold):
                goodNeighbors.append(move)
        return goodNeighbors #State farm joke goes here

    def isAtGoalPosition(self, currentLoc):
        """
        Takes in a tuple of (x, y) that is the current robot location.
        Returns true if we're there, false otherwise.
        """
        if tools.distFormula(currentLoc, self._goal_pos) < 0.1: # Maybe this error bar should go out globally, or take it in.
            return True
        else:
            return False


    def isAtGoalAngle(self, currentAngle):
        """
        Takes in the robot's current angle in radians, tests if that angle is the goal angle.
        """
        if abs(currentAngle - self._goal_[2]) < 0.26: #Like 14 and a half degrees
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
        return self.current_x,self.current_y


    def getRobotAngle(self):
        return self.current_theta


    def main(self):
        try:
            pass
        except rospy.ROSInterruptException:
            pass



















