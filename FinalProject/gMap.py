__author__ = 'Troy Hughes'

import rospy
import tools
import tf
import rVizPainter as rvptr
from turtleExceptions import FrontierException
import FullMapExplorer as FME

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
        self.FE = FME.FME(0.05,0.352)


    def doneSetup(self):
        """
        This function checks if the map and robot's position are initialized.
        """
        if self._mapSet is False:#If the map is not yet set up.
            print "Map is not set"
            return False
        if self.current_x is None or self.current_y is None or self.current_theta is None:#if any of the robot's x y theta is unset
            print "Waiting on location data for robot"
            self._updateLocation()
            return False
        print 'gMap is properly configured'#prints that the map and robot are correctly configured.
        return True

    def updateMap(self, msg):
        """
        This is the function that is called whenever the "/map" topic is published to
        :param msg:
        :return:
        """
        self._map = tools.lMaptoLLMap(msg.data,
                                      msg.info.height,
                                      msg.info.width)#Updates the map with the map data, height and width.
        self._max_h = msg.info.height#Updates max height
        self._max_w = msg.info.width#Updates max width
        self._pose = msg.info.origin#Updates the pose to be origin
        self._mapSet = True
        self._updateLocation()#Updates the location


    def addValue(self, x, y, value):
        """
        Adds a value to the map at a specific point on the map.
        :param value: the value of the specific map node observed.
        :param x, y: The values along the map to actually place that value at.
        """
        if self._mapSet:
            raise RuntimeError("Trying to add with no map")
        try:
            self._map[y][x] = ((self._map[y][x]) * (0.25) + value * (0.75)) / 2.0 #Updates the map with a weight for new vs old value at that point.
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
        self._updateLocation()
        raise NotImplementedError("Goal storing is not implemented, currently just updates location")

    def _updateLocation(self):
        """
        Updates the robot's location on the map.
        """
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
        print "Your robots current location is ",self.current_x,self.current_y,self.current_theta
        return True



    def getNextFrontier(self, Verbose = False):
        """
        :return: Returns a pose on the map in Meters for the robot to drive to
        """

        if Verbose:
            print "Trying to find a new Frontier"

            ## Get the list of frontiers that exist on the map
            frontierList, dilatedMap = self.FE.getFrontierList(self._map)
            print ""
            print "Fontiers found: "
            for n in frontierList:
                print ""
                print n
            print ""
        else:
            frontierList, dilatedMap = self.FE.getFrontierList(self._map)

        ## Get the map location in grid cells of the frontier to travel to (can expand to have multiple heuristics for this)
        if len(frontierList) == 0:
            print frontierList
            raise FrontierException("The number of frontiers you have are zero")

        ## Pickes the frontier based off the passed heuristic function.
        mapLocationGridCells = self.FE.pickFrontier(frontierList, self.FE.frontierSize)

        ## Ensures the point that you're going to is somewhat viable {{ Does not performe an a* alrogirhm
        ## as of (Dec, 10) only checks for valid points.
        safeMapLocation = self.FE.findSafePoint(mapLocationGridCells,
                                                (self.current_x,self.current_y),
                                                dilatedMap)

        ## Make a big signal for rViz to make it easier to see.
        signal = []
        for neighbor in tools.getNeighbors(safeMapLocation[0],safeMapLocation[1],dilatedMap,threshold=50):
            x,y = neighbor
            signal.extend(tools.getNeighbors(x,y,dilatedMap,threshold=50))
            signal.append(neighbor)

        ## Paint the location that you're moving to
        self.painter.paint('/testSquares',signal)

        ## Return the map location in meters so that the pose can just be gone to
        return self.FE.mapLocationMeters(safeMapLocation, self.current_x, self.current_y)



    def getRobotPosition(self):#THIS IS THE PART WHERE FRANCE INVADES
        """
        Makes sure that the robot's x and y are known, and then returns them if it is.
        """
        if self.current_x is None:
            print "The positions aren't set", self.current_x, self.current_y
            raise RuntimeError("Positions are still not set!")
        return self.current_x, self.current_y

    def getRobotAngle(self):
        """
        Returns the robot's theta.
        """
        return self.current_theta

    def main(self):
        try:
            pass
        except rospy.ROSInterruptException:
            pass


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