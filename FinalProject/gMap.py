__author__ = 'Troy Hughes'

import rospy
import tools
import tf
from nav_msgs.msg import OccupancyGrid
import rVizPainter as rvptr
from turtleExceptions import FrontierException
import FullMapExplorer as FME
import PathPlanner as PP


class gMap():
    def __init__(self, name, goalTopic):
        """
        Takes the name of the object and the name of the topic.
        :param name: <string>
        :param goalTopic: <ROSTOPIC formatted String> Ex: '/someTopic'
        :return:
        """
        # Location Information
        self.current_x, self.current_y, self.current_theta = None, None, None
        self._currentSet = False

        # Map Information
        self._map = [[]]
        self._height = 0
        self._width = 0
        self._pose = None
        self._mapSet = False
        self._map_list = tf.TransformListener()
        self._DMap = rospy.Publisher('/DAMPA', OccupancyGrid, queue_size=1)
        self._max_h = None  # unknown value, should ALWAYS be read in. We should really have error checking for this.
        self._max_w = None  # ^
        self._res = None  # ^

        # Contained Classes ####
        #   Visualiser Class  ##
        self.painter = rvptr.rVizPainter(name + " Painter", 0.05)
        self.goalTopic = goalTopic
        self.painter.addPainter(self.goalTopic)
        self.painter.addPainter("/FRONTIER")
        self.painter.addPainter("/PATH")

        #   Frontier Explorer #
        self.FE = FME.FME(0.05, 0.352)

        #   Path Planner      #
        self.PP = PP.PathPlanner(name+" Path Planner")

    def doneSetup(self):
        """
        This function checks if the map and robot's position are initialized.
        """
        if self._mapSet is False: # If the map is not yet set up.
            print "Map is not set"
            return False
        if self.current_x is None or self.current_y is None or self.current_theta is None:  # if any of the robot's x y theta is unset
            print "Waiting on location data for robot"
            self._updateLocation()
            return False
        print 'gMap is properly configured'  # prints that the map and robot are correctly configured.
        return True

    def updateMap(self, msg):
        """
        This is the function that is called whenever the "/map" topic is published to
        :param msg:
        :return:
        """
        newMap = tools.lMaptoLLMap(msg.data, msg.info.height, msg.info.width)
        # Updates the map with the map data, height and width.

        for i in xrange(2):  # Dilates the map by a semi-arbitrary value.
            newMap = tools.dialateOccupancyMap(newMap, len(self._map[0]),len(self._map))
        self._map = newMap
        self._max_h = msg.info.height  # Updates max height
        self._max_w = msg.info.width  # Updates max width
        self._res = msg.info.resolution
        self._pose = msg.info.origin  # Updates the pose to be origin
        self._mapSet = True

        msg.data = tools.llMaptoLMap(self._map, self._max_h, self._max_w)
        self._DMap.publish(msg)
        self._updateLocation()#Updates the location

    def _updateLocation(self):
        """
        Updates the robot's location on the map. :: All updated values will be in the map frame values.
        """
        try:
            self._map_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1)) #Used 1 instead of 0.5
        except tf.Exception:
            print "Waiting for transform failed"
            self.current_x = 0
            self.current_y = 0
            self.current_z = 0
            return False

        (p, q) = self._map_list.lookupTransform("map", "base_footprint", rospy.Time(0))
        x, y, z = p
        self.current_theta = tools.normalizeTheta(q)
        self.current_x = (tools.globalToMap(x, self._pose.position.x, self._res))
        self.current_y = (tools.globalToMap(y, self._pose.position.y, self._res))
        self.current_z = (tools.globalToMap(z, self._pose.position.z, self._res))
        self._currentSet = True
        # print "Your robots current location is ",self.current_x,self.current_y,self.current_theta
        return True



    def getFrontierList(self, Verbose = False):
        """
        This function get's the frontiers that are to be explored and ensures that there are not no lists.

        :return: Returns the list of frontiers checked to ensure that it is not an empty list
        """

        if Verbose:
            print "Trying to find a new Frontier"

            # Get the list of frontiers that exist on the map
            frontierList, dilatedMap = self.FE.getFrontierList(self._map)
            print 'The following will display the frontiers discovered'

            for i,frontier in enumerate(frontierList):
                self.paintFrontier(frontier)
                print "Sent frontier to be painted",i
                print "\t-- The first point on this frontier is: ", frontier[0]
                rospy.sleep(1.5)

        else:
            frontierList, dilatedMap = self.FE.getFrontierList(self._map)

        if len(frontierList) == 0:
            raise FrontierException("The number of frontiers y ou have is zero")

        return frontierList

    def getNextFrontier(self, verbose=True):
        """
        Gets the next frontier!
        """
        try:
            frontierList = self.getFrontierList(verbose)
        except FrontierException,e:
            print "You are done exploring"
            raise FrontierException("getFrontierList threw a frontier exception")

        # Picks the frontier based off the passed heuristic function.
        currentMapPosition = (self.current_x,self.current_y)

        frontierQueue = self.FE.sortFrontiers(frontierList, self.FE.frontierSize, self._map)

        nextFrontier = None
        frontierPoint = None
        while not frontierQueue.empty() and not rospy.is_shutdown():
            queueItem = frontierQueue.get()
            p, frontierInfo = queueItem
            frontierPoint, frontier = frontierInfo
            x,y = frontierPoint
            if self._map[int(y)][int(x)] == 100:
                raise RuntimeError("Fuck everything")
            decision = self.PP.canTravelTo(self._map,currentMapPosition,frontierPoint)
            print "Your decision is: ",decision

            if decision:
                path = self.PP._getPath(self._map, currentMapPosition, frontierPoint) # @TODO: Don't access protected info
                nextFrontier = frontierPoint
                self.paintFrontier(frontier)
                self.paintPath(path)
                break
        if nextFrontier is None:
            print "You are done exploring"
            raise FrontierException("nextFrontier is None, therefore you are done...")

        if verbose: print "Your frontier point is: " ,frontierPoint
        if verbose: print "The global frontier popint is: ", self.convertMapToGlobal(frontierPoint)

        print "Before running: ", frontierPoint

        travelPoint = self.PP.frontierToTravelPoint(self._map,currentMapPosition,frontierPoint)
        print "Frontier Point is: ", frontierPoint
        print "Travel Point is: ", travelPoint
        self.paintGoal(travelPoint)

        if verbose:
            print "Your Travel point is:", travelPoint
            print "Your frontier midpoint is: ", frontierPoint

        globalPoint = self.convertMapToGlobal(travelPoint)
        x, y = globalPoint
        return x, y, False

    def setLegalMoveBaseGoal(self,point, threshold=65):
        """
        This function finds a legal place to make sure that the robot is capable of moving to the location

        :param Point : <x,y> touple representing the locaiton in the gMap
        """
        frontierMap = tools.dialateOccupancyMap(self._map, len(self._map), len(self._map[0]))
        for i in xrange(3):
            frontierMap = tools.dialateOccupancyMap(frontierMap, len(frontierMap), len(frontierMap[0]))

        px, py = point
        self.painter.paintGoal(self.goalTopic, self.convertMapToGlobal(point))

        if threshold > frontierMap[py][px] > -1:
            return point

        raise FrontierException("The frontier you have selected is not within your visiting capabilities")

    def paintFrontier(self, frontier):
        """
        Publishes to the /FRONTIER topic to paint a frontier.
        :param frontier: The frontier to paint.
        """

        globalFrontier = []

        for f in frontier:
            globalFrontier.append(self.convertMapToGlobal(f))

        self.painter.paint('/FRONTIER', globalFrontier)

    def paintPath(self, path):
        """
        Publishes to the /PATH topic to paint a path given.
        :param path: The path to paint.
        """

        pathToPaint = []

        for waypoint in path:
            pathToPaint.append(self.convertMapToGlobal(waypoint))

        self.painter.paint('/PATH', pathToPaint)


    def paintGoal(self, goal, isGlobal=False):
        if isGlobal: self.painter.paintGoal(self.goalTopic, goal)
        else: self.painter.paintGoal(self.goalTopic, self.convertMapToGlobal(goal))  # # Default case




    def convertMapToGlobal(self,point):
        """
        Wrapper for the map points to global points
        """
        return tools.mapPointToGlobal(point, (self._pose.position.x, self._pose.position.y, None), self._res)

    def convertGlobalToMap(self, point):
        """
        Wrapper for global points to map points
        """
        return tools.globalPointToMap(point, (self._pose.position.x, self._pose.position.y, None), self._res)




    def getRobotPosition(self):  # THIS IS THE PART WHERE FRANCE INVADES, who wrote this comment?
        """
        Makes sure that the robot's x and y are known, and then returns them if it is.
        """
        if self.current_x is None:
            print "The positions aren't set", self.current_x, self.current_y
            raise RuntimeError("Positions are still not set!")
        return self.convertMapToGlobal((self.current_x,self.current_y))

    def getRobotAngle(self):
        """
        Returns the robot's theta.
        """
        return self.current_theta

    def main(self):
        """
        Because we need a main I guess.
        """
        try:
            pass
        except rospy.ROSInterruptException:
            pass

    def getNavFrontiers(self, verbose=False):
        """
        Gets the list of frontiers, then makes the queue of that list of frontiers.

        :return: Queue This is the queue (Using A* heuristics I believe?) of frontiers to explore.
        """
        try:
            flist = self.getFrontierList(verbose)
        except FrontierException,e:
            print 'getFrontierList has thrown an acception'
            raise e

        cpos = (self.current_x, self.current_y)

        fq = self.FE.queueMaker(flist, self._map, cpos, verbose)
        return fq






def testPainter():
    """
    This function is for testing the painter. Z
    """
    rospy.init_node("someName")
    a = gMap("Name")
    a.painter.addPainter("/SomeTopic")
    raw_input("Enter when ready")
    # paintList = []
    # for i in xrange(10):
    #     for j in xrange(100):
    #         paintList.append((i,j))
    # a.painter.paint("/SomeTopic",paintList)
    a.painter.paintGoal('/SomeTopic',(1,1))

    rospy.spin()
# testPainter()