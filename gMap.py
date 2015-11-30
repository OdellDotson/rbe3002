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

        self._map_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid , self._updateMap)







    def _updateMap(self,msg):
        self._map = tools.lMaptoLLMap(msg.data,
                                      msg.info.height,
                                      msg.info.weight)
        self._max_h = msg.info.height
        self._max_w = msg.info.weight
        self._pose = msg.info.origin
        self._mapSet = True




    def addValue(self, x,y,value):
        if self._mapSet:
            raise RuntimeError("Trying to add with no map")
        try:
            self._map[y][x] = ((self._map[y][x])*(0.25) + value*(0.75))/2.0
        except IndexError,e:
            print "Error in addValue"
            print e


    def storeGoal(self, goalX, goalY, goalTheta):
        """
        THis stores the goal and get's the current location of the robot in the map.

        :param goalX: x location of the goal in the grid frame
        :param goalY: y location of the goal in the grid frame
        :param goalTheta: angle of the goal in the grid frame (no difference)
        :return: self._goal_, self._current_ which is an : x,y,theta location of the goal and an x,y location of the
            robot.
        """
        print "Getting ready to store your goal"
        self.goalX,self.goalY,self.goaltheta = goalX,goalY,goalTheta
        self._updateLocation()

        print (int(self.goalY), int(self.goalX))

        goal = (self.goalX, self.goalY)
        """ Stores the goal for a* and the goal for xytheta"""
        self._goal_pos = (self.goalX, self.goalY)
        self._goal_ = (self.goalX, self.goalY, self.goaltheta)
        self._updateLocation() ## Run a second time for map updating asurence

        self._goalSet = True
        return self._goal_, (self.current_x,self.current_y,self.current_theta)


    def _updateLocation(self):
        try:
            self._map_list.waitForTransform('map', 'base_footprint', rospy.Time.now(), rospy.Duration(0.5))
        except tf.Exception:
            print "Waiting for transform failed"
            return False

        (p,q) = self._map_list.lookupTransform("map","base_footprint",rospy.Time.now())
        x,y,z = p
        self.current_theta = tools.normalizeTheta(q)

        self.current_x = tools.gmapifyValue(x)
        self.current_y = tools.gmapifyValue(y)
        self.current_z = tools.gmpaifyValue(z)
        self._currentSet = True
        return True

    def repaint(self):
        """
        This method is for repainting on rViz. Defining it as doing nothing to not break turtlebot.py
        :return:
        """
        return

    def getNextWaypoint(self):
        if self._mapSet is False or self._currentSet is False or self._goalSet is False:
            print "Mapset:", self._mapSet, "CurrentSet", self._currentSet, "GOALSET", self._goalSet
            raise RuntimeError("Trying to get waypoints on unset map/location/goal")
        nodePath = self.getWaypoint()
        if len(nodePath) >= 2:return nodePath[1]
        else:return []

    def getWaypoint(self):
        self._updateLocation()
        start = (self.current_x,self.current_y)

        pathToNodify = self.getPath(start) # Get path from A*
        nodePath = []  # What will become the path of only relevant nodes.
        prevSlope = 1337.0  # Create an impossible to match previous slope

        for node in range (0, len(pathToNodify)-1): # For every node in the path created by A*:
            if pathToNodify[node] != self._goal_pos: # Do not try to calculate a slope for the last node
                #calculate current slope, from current position to next position.
                deltax = 1.0*(pathToNodify[node][0]-pathToNodify[node+1][0])#Calcualte delta x based on node and the node ahead
                deltay = 1.0*(pathToNodify[node][1]-pathToNodify[node+1][1])#calculate delta y
                currentSlope = deltax/(deltay+0.000001) #So that we don't have devide by 0 errors. Dunno how else to fix
                if currentSlope != prevSlope: # If we see a slope change
                    nodePath.append(pathToNodify[node])# Add the new node
                    prevSlope=currentSlope # Update th slope

        #Append the very last node, so that we have a complete path.
        nodePath.append(pathToNodify[len(pathToNodify)-1])
        # self.waypoint_list = tools.publishListfromTouple(nodePath)
        # self._repaint_map(path=True)
        # print nodePath
        return nodePath


    def getPath(self, start):
        """
        This function uses A* to generate a from start to end and then returns it.
        :param start:
        :return: list of points (x,y touples) that is the path to take to the goal
        """
        came_from = self.aStarSearch(start)          ## Get dictionary from astar
        path = [self._goal_pos]                      ## Initialize path

        ## Prime the statements for iterating to the end of the path
        current_location = self._goal_pos
        prev = came_from[current_location]
        path.append(prev)
        current_location = prev

        ## Traverse the tree from leaves to trunk
        while prev is not None:
            prev = came_from[current_location]
            if prev is None:
                continue
            path.append(prev)
            current_location = prev

        path.reverse()                              ## Reverse the list so it goes from start to finish

        ## Convert the path list to a publishable list so we can view the path in rViz
        # self.path_list = tools.publishListfromTouple(path)
        # self._repaint_map(path=True)                ## Repaint the map so we can see it.

        return path

    def aStarSearch(self, start):
        """
        This function generates a dictionary of paths where the shortest path can be found through
        traversing from the goal back to the start.

        This version of A* uses tools.distFormula as the heuristic. Could be changed out.

        :param start: (x,y) touple of the location in the grid.
        :return: Dictionary of <end point> : <starting Point> as key,value
        """
        frontier = Queue.PriorityQueue()
        frontier.put((0, start))            ## Put takes a touple of priority, value

        came_from = {}
        cost_so_far = {}

        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current_touple = frontier.get()                                             ## Returns the (priority, (x,y)
            _,current = current_touple                                                  ## sets current to (x,y)
            x,y = current

            if x == self.goalX and y == self.goalY:
                break #We're there!

            #unpack current
            neighbors = self.getNeighbors(x,y)
            # print neighbors
            for next in neighbors:                                         ## Get list of touples (points)
                # print "AStar",next,current
                cost_next = cost_so_far[current] + tools.distFormula(current, next)
                if next not in cost_so_far or cost_so_far[next] > cost_next:
                    if next not in cost_so_far:
                        frontier.put((cost_next + tools.distFormula(next, self._goal_pos), next))## Put takes a touple of priority, value
                        nx,ny = next
                        ## THis is where explored nodes would be added to a list for painting.
                        # self.explored_nodes_list.append(tools.makePoint(nx,ny,0))
                        # self._repaint_map()
                    cost_so_far[next] = cost_next
                    came_from[next] = current


        if frontier.empty():
            print "It's empty!!!!!"
            rospy.sleep(1)
            print "CAME FROM:", came_from
            print "GOAL", self._goal_
            print "START", start
            raise RuntimeError("A* cannot find the path for some reason")
        return came_from

    def getNeighbors(self,x,y,threshold=99):
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
        if (tools.distFormula(currentLoc, self._goal_pos) < 0.1): # Maybe this error bar should go out globally, or take it in.
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
        return (self.current_x,self.current_y)

    def getRobotAngle(self):
        return self.current_theta





    def main(self):
        try:
            pass
        except rospy.ROSInterruptException:
            pass



















