__author__ = 'Odell Dotson'

# Subclass Information
from log_base import log_base
from communicator import communicator

class map(log_base):
    def __init__(self):
        rospy.init_node(name)
        self._name_ = name
        self.initMap()
        pass

    def initMap(self):
        pass
        #  Create a map, empty of objects, places the robot at 0,0 initially

    def createPath(self):
        pass
        #  Get data from (somewhere?) and create a path (on top of the map/using the map) using A*


    def updateMap(self):
        pass
        #  Read in the robot's most recent position
        #  Update the map based on new obsticles that the robot sees.
        #  Perhaps a queue of "newly seen" things sensed by the robot that the map filters in and creates new targets...
        #  ... to send in.


    def nextNode(self):
        pass
        #  Robot calls this against map to get the next target once it has decided it is close enough to the ...
        #  ... previous target (or enough time has passed?)



    def main(self):
        try:
            pass
        except rospy.ROSInterruptException:
            pass

Map = map("world")
