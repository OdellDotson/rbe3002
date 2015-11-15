__author__ = 'Odell Dotson'

import rospy
import tools
import Queue
from nav_msgs.msg import GridCells
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point

# import logging

# Subclass Information
from communicator import communicator

class map():
    def __init__(self, name):
        self._map = None ## initialized first to avoid hanging
        self._name_ = name

        self._map_sub = rospy.Subscriber('/map', OccupancyGrid , self._updateMap)

        self._walls = rospy.Publisher('/walls', GridCells, queue_size=1)
        self._explored_nodes = rospy.Publisher('/explored_nodes', GridCells, queue_size=1)
        self._not_explored_nodes = rospy.Publisher('/not_explored_nodes', GridCells, queue_size=1)
        self._path = rospy.Publisher('/path', GridCells, queue_size=1)

        # logging.info("Waiting for the _map to contain valid informaiton.")

        while self._map is None:
            rospy.sleep(0.1)
            print "Map is none..."
            continue

        # logging.info("Begining to populate the colored map")
        for i in xrange(20):
            self._start_populate()
        # logging.info("Colored map populated 20x to ensure no ROS errors")
        print "Exiting"

    def initMap(self):
        pass
        #  Create a map, empty of objects, places the robot at 0,0 initially

    def createPath(self):
        pass
        #  Get data from (somewhere?) and create a path (on top of the map/using the map) using A*
    def _start_populate(self, threshold =99):
        # logging.info("Configuring for publishing")
        grid = self._map

        wall_cells = GridCells()
        wall_cells.header.frame_id = 'map'
        wall_cells.cell_width = 0.3; wall_cells.cell_height = 0.3

        not_explored = GridCells()
        not_explored.header.frame_id = 'map'
        not_explored.cell_width = 0.3; not_explored.cell_height = 0.3

        wall_list =[]; not_explored_list = []

        k=0
        for i in range(1,self._height): #height should be set to hieght of grid
            k=k+1
            for j in range(1,self._width): #height should be set to hieght of grid
                k=k+1

                point=Point()
                point.x=j*0.3
                point.y=i*0.3
                point.z=0

                if (grid[k] == 100):
                    wall_list.append(point)
                else:
                    not_explored_list.append(point)
        wall_cells.cells = wall_list
        not_explored.cells = not_explored_list

        self._walls.publish(wall_cells)
        rospy.sleep(0.1)
        self._not_explored_nodes.publish(not_explored)
        rospy.sleep(0.1)

    def _updateMap(self, msg):
        self._map=msg.data
        self._height=msg.info.height
        self._width=msg.info.width
        self._pose=msg.info.origin # Do we actually need the pose? If not, remove
        #print self._map

    def storeGoal(self, msg):
        self.x = msg.point.x
        self.y = msg.point.y
        self.theta=tools.normalizeTheta((msg.quaternion.x,msg.quaternion.y,msg.quaternion.z,msg.quaternion.w))
        goal = (self.x, self.y)
        goalTheta = (self.x, self.y, self.theta)

    def getNeightbors(self,x,y,threshold=99):#Only returns good neighbors
        goodNeighbors = []
        if(x>self._width or x < 0 or y>self._height or y < 0):
            raise ReferenceError("getNeighbors out of bound error on x or y coordinate.")
        #Goes through the values, ignores self
        for i in range (x-1, x+2):#Itterate though x locations
            for j in range(y-1, y+2):#itterate through y locations
                if (i is not x and j is not y) and not (x>self._width or x < 0 or y>self._height or y < 0):#If we're not looking at our current location and we're not looking out of range
                    if(self._data[i][j] < threshold):#If the neighbor is a valid place to go to
                        goodNeighbors.append((i,j))
        return goodNeighbors #State farm joke goes here

    def aStarSearch(self, start):
        frontier = Queue.PriorityQueue
        frontier.put(0, start)

        came_from = {}
        cost_so_far = {}

        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break #We're there!

            x,y = current
            for next in self.getNeighbors(x,y):
                cost_next = cost_so_far[current] + tools.distFormula(next, current)
                if next not in cost_so_far or cost_so_far[next] > cost_next:
                    cost_so_far[next] = cost_next
                    frontier.put(cost_next + tools.distFormula(next, goal), next)
                    came_from[next] = current
        return came_from

    def getPath(self, start):
        came_from = aStarSearch(start)#A*, create path of nodes, returns the came_from dictionary
        path = []
        prev = came_from[goal]
        path.append(prev)
        while current is not start:
            current = came_from[prev]
            path.append(current)
        return path.reverse()


    def main(self):
        try:
            pass
        except rospy.ROSInterruptException:
            pass

