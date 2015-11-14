__author__ = 'Odell Dotson'

import rospy
import tools
from nav_msgs.msg import OccupancyGrid

# Subclass Information
from log_base import log_base
from communicator import communicator

class map(log_base):
    def __init__(self, name):
        self._name_ = name
        self._map_sub = rospy.Subscriber('/map', OccupancyGrid , self._updateMap)
        self.initMap()
        pass

    def initMap(self):
        pass
        #  Create a map, empty of objects, places the robot at 0,0 initially

    def createPath(self):
        pass
        #  Get data from (somewhere?) and create a path (on top of the map/using the map) using A*


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




    def nextNode(self):
        pass
        #  Robot calls this against map to get the next target once it has decided it is close enough to the ...
        #  ... previous target (or enough time has passed?)



    def main(self):
        try:
            pass
        except rospy.ROSInterruptException:
            pass

