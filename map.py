__author__ = 'Odell Dotson'

import rospy
import math
import tools
import Queue
import tf
from nav_msgs.msg import GridCells
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point

# import logging

# Subclass Information
from communicator import communicator

class map():
    def __init__(self, name):
        self._startup = False ## initialized first to avoid hanging
        self._name_ = name

        self.wall_list = []
        self.explored_nodes_list = []
        self.not_explored_nodes_list = []
        self.path_list = []
        self.waypoint_list = []


        self._map_sub = rospy.Subscriber('/map', OccupancyGrid , self._updateMap)

        self._walls = rospy.Publisher('/walls', GridCells, queue_size=1)
        self._explored_nodes = rospy.Publisher('/explored_nodes', GridCells, queue_size=1)
        self._not_explored_nodes = rospy.Publisher('/not_explored_nodes', GridCells, queue_size=1)
        self._waypoints = rospy.Publisher('/waypoints', GridCells, queue_size=1)
        self._path = rospy.Publisher('/path', GridCells, queue_size=1)

        self._map_list = tf.TransformListener()

        # logging.info("Waiting for the _map to contain valid informaiton.")

        while self._startup is False:
            rospy.sleep(0.1)
            print "Map is none..."
            continue

        # logging.info("Begining to populate the colored map")
        for i in xrange(20):
            self._start_populate()
        print "Map instantiated properly"
        # logging.info("Colored map populated 20x to ensure no ROS errors")


    def _start_populate(self, threshold =99):
        """
        This function publishes the walls and unexplored maps to the 'self._wall' and 'self._not_explored_nodes'
        publishers. This gives the map a basis to view what is occuring in realtime.

        This is used as a startup function, and as a base paint function.

        :param threshold: This is the value that is used to discern a wall vs. unknown space
        :return: None
        """

        # logging.info("Configuring for publishing")
        grid = self._map

        wall_cells = tools.makeGridCells('map',0.3,0.3)
        not_explored = tools.makeGridCells('map',0.3,0.3)

        self.wall_list =[]; self.not_explored_list = []

        for i in range(0,self._height): #height should be set to hieght of grid
            for j in range(0,self._width): #height should be set to hieght of grid
                point = tools.makePoint(j,i,0)
                if (self._map[i][j] == 100):
                    self.wall_list.append(point)
                else:
                    self.not_explored_list.append(point)
        wall_cells.cells = self.wall_list
        not_explored.cells = self.not_explored_list

        self._walls.publish(wall_cells)
        rospy.sleep(0.1)
        self._not_explored_nodes.publish(not_explored)
        rospy.sleep(0.1)

    def _updateMap(self, msg):
        """
        This is the callback for when the '/map' topic publishes changes.

        The function is smart enough to know when it is the first vs. when the map is updated.

        :param msg:
        :return: None
        """
        self._map = []
        for i in xrange(msg.info.height):
            new_list = []
            new_list.extend(msg.data[i*msg.info.width:(i+1)*msg.info.width])
            self._map.append(new_list)
        self._map = tools.dialateOccupancyMap(self._map)

        self._height=msg.info.height
        self._width=msg.info.width
        self._pose=msg.info.origin
        if self._startup is True:
            self._repaint_map(baseMap=True)
        self._startup = True



    def _repaint_map(self, baseMap=False, path=False):
        """
        This repaints the map based off the path from A* and the explored regions.

        This is also run if the map is updated through 'self._updateMap(self, msg)'

        :return:
        """
        if baseMap:
            self._start_populate()
            self._path.publish(tools.makeGridCells('/map',0.3,0.3,self.path_list))
        if path:
            self._path.publish(tools.makeGridCells('/map',0.3,0.3,self.path_list))
        for i in xrange(10):
            self._explored_nodes.publish(tools.makeGridCells('/map',0.3,0.3,self.explored_nodes_list))
            self._waypoints.publish(tools.makeGridCells('/map',0.3,0.3,self.waypoint_list))



    def storeGoal(self, msg):
        """
        This is the callback that is attached in the parent class. This catches rVis's 2D Nav Pose buttons

        :param msg:
        :return:
        """
        self.explored_nodes_list = []
        self.waypoint_list = []
        self.path_list = []


        (p,q) = self._map_list.lookupTransform("map","base_footprint",rospy.Time(0))
        self.current_x,self.current_y, self.current_z = p
        self.current_theta = tools.normalizeTheta(q)


        self.goalX = int(math.floor(msg.pose.position.x/0.3)) -1
        self.goalY = int(math.floor(msg.pose.position.y/0.3))

        while (self._map[self.goalY][self.goalX] != 0):
            self.goalX = self.goalX+1

        self.goaltheta=tools.normalizeTheta((msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w))
        goal = (self.goalX, self.goalY)
        """ Stores the goal for a* and the goal for xytheta"""
        self._goal_pos = (self.goalX, self.goalY)
        self._goal_ = (self.goalX, self.goalY, self.goaltheta)
        print self._goal_pos


        # print 'Getting current location:',self.current_x, math.floor(self.current_x), int(math.floor(self.current_x))
        self.getWaypoint((int((math.floor(self.current_x/0.3)))-1,
                     int(math.floor(self.current_y/0.3))))

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
                    if next not in cost_so_far
                        frontier.put((cost_next + tools.distFormula(next, self._goal_pos), next))## Put takes a touple of priority, value
                        nx,ny = next
                        self.explored_nodes_list.append(tools.makePoint(nx,ny,0))
                        self._repaint_map()
                    cost_so_far[next] = cost_next
                    came_from[next] = current


        if frontier.empty():
            print "It's empty!!!!!"
            rospy.sleep(1)
        return came_from


    def getWaypoint(self, start):
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
        self.waypoint_list = tools.publishListfromTouple(nodePath)
        self._repaint_map(path=True)
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
        self.path_list = tools.publishListfromTouple(path)
        self._repaint_map(path=True)                ## Repaint the map so we can see it.

        return path


    def main(self):
        try:
            pass
        except rospy.ROSInterruptException:
            pass

