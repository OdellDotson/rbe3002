__author__ = 'Odell Dotson'


class map(log_base):
    def __init__(self):
        rospy.init_node(name)
        self._name_ = name
        self.initMap()
        pass

    def initMap(self):
        pass
        #  Create a map, empty of objects, places the robot at 0,0 initially


    def updateMap(self):
        pass
        #  Read in the robot's most recent position
        #  Update the map based on new obsticles that the robot sees.
        #  Perhaps a queue of "newly seen" things sensed by the robot, that the map filters in and creates new targets...
        #  ... to send in.





    def sendNextNode(self):
        pass
        #  Send the next node from our A*?



    def main(self):
        try:
            pass
        except rospy.ROSInterruptException:
            pass

Map = map("world")
