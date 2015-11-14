__author__ = 'Odell Dotson'

import rospy
import tools
import Queue
from nav_msgs.msg import GridCells
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point

# Subclass Information
from log_base import log_base
from communicator import communicator

class map(log_base):
    def __init__(self, name):
        self._name_ = name
        self._map_sub = rospy.Subscriber('/map', OccupancyGrid , self._updateMap)
        self._color_test = rospy.Publisher('/testPub', GridCells, self._testColor,queue_size=1)

    def initMap(self):
        pass
        #  Create a map, empty of objects, places the robot at 0,0 initially

    def createPath(self):
        pass
        #  Get data from (somewhere?) and create a path (on top of the map/using the map) using A*
    def _testColor(self):
        print "In Test color"
        grid = self._map

        k=0
        cells = GridCells()
        cells.header.frame_id = 'map'
        cells.cell_width = 0.3 # edit for grid size .3 for simple map
        cells.cell_height = 0.3 # edit for grid size

        for i in range(1,self._height): #height should be set to hieght of grid
            k=k+1
            for j in range(1,self._width): #height should be set to hieght of grid
                k=k+1
                #print k # used for debugging
                if (grid[k] == 100):
                    point=Point()
                    point.x=j*0.3 # edit for grid size
                    point.y=i*0.3 # edit for grid size
                    point.z=0
                    cells.cells.append(point)
        #print cells # used for debugging
        self._color_test.publish(cells)

    def _updateMap(self, msg):
        self._map=msg.data
        self._height=msg.info.height
        self._width=msg.info.width
        self._pose=msg.info.origin # Do we actually need the pose? If not, remove
        self._testColor()
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

            





    def nextNode(self):
        pass
        #  Robot calls this against map to get the next target once it has decided it is close enough to the ...
        #  ... previous target (or enough time has passed?)



    def main(self):
        try:
            pass
        except rospy.ROSInterruptException:
            pass

