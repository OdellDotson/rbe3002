__author__ = 'Troy Hughes'

import rospy
import tf
import tools
import math
from nav_msgs.msg import OccupancyGrid

class localMap():
    """
    This class takes things in in the global frame and returns them in the global frame.
    The global framed values will be mapified, therefore may need to be demapified.

    """
    def __init__(self, name):
        self._name_ = name
        self._test_pub = rospy.Publisher('test_costmap', OccupancyGrid, queue_size=1)

        self._map_tfListener = tf.TransformListener()
        self.location = None
        self._max_w = None
        self._max_h = None
        self._x, self._y = None, None
        self._doneIter = False


    def storeCostmap(self, msg, gMap=True):
        try:
            self._map_tfListener.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(0.2))
        except tf.Exception:
            print "TF library chunkin out broski"
            return
        self._mapLL = tools.lMaptoLLMap(msg.data,
                                      msg.info.height,
                                      msg.info.width)
        self._mapL = msg.data

        # self._max_w = 37#msg.info.width
        # self._max_h = 37#msg.info.height
        self._max_w = msg.info.width
        self._max_h = msg.info.height
        self._pose = msg.info.origin.position

        try: ## Try to get the transform to the location of the robot.
            (p,q) = self._map_tfListener.lookupTransform("map","base_footprint",rospy.Time.now())
        except tf.Exception:
            print "Python can't read your future, calm down"
            return
        if gMap: ## IF this is being run on the gMap
            self._ox,self._oy,_ = p                                 ## Get the position of the robot from the map to base footprint transform
            self._ox = tools.gmapifyValue(self._ox + self._pose.x)   ## create a '/map' grid value from the sum of the robot location and the offset to the x min of the costmap
            self._oy = tools.gmapifyValue(self._oy + self._pose.y)   ## create a '/map' grid value from the sum of the robot location and the offset to the y min of the costmap

            self._reducedMap = self._mapLL
            self._reduce_w = self._max_w
            self._reduce_h = self._max_h
        else: ## IF this is being run in the simulator
            ## Set the offsets in the map space
            self._ox,self._oy,_ = p                                 ## Get the position of the robot from the map to base footprint transform
            self._ox = tools.mapifyValue(self._ox + self._pose.x + 1)   ## create a '/map' grid value from the sum of the robot location and the offset to the x min of the costmap
            self._oy = tools.mapifyValue(self._oy + self._pose.y)   ## create a '/map' grid value from the sum of the robot location and the offset to the y min of the costmap

            ## Reduce the resolutino on this map:
            self._reducedMap, self._reduce_w, self._reduce_h = self._shrinkTwo()
            # print self._reducedMap


    def _shrinkTwo(self):
        """
        This is a hardcoded piece of shit. Hate it. 

        This function takes the stored Costmap and dialates all the pixles
        on it by 1 in the left and right direction. This means that if the
        whole image 'was' an 60x60 image, this will reduce it to a 20X20 image

        NOTE: This is done because the '/map' resolution is 0.3 and the '/costmap' resolution is 0.05.
        This will make them compariable.
        :return: List of Lists as a Map
        """
        otherMap = []
        temp = []
        yvalslist = []
        counter = 0
        num_x = 0
        num_y = 0
        for y,row in enumerate(self._mapLL):    ## For every row and every value in every row
            yvals = []
            for x,val in enumerate(row):
                counter = counter + val         ## Accumulate a value for each 'row' shrinker
                if x % 6 == 5:                  ## if this is the 3rd value, append it to the yvals list and reset counter
                    yvals.append(counter)
                    counter = 0
                num_x = x                       ## use this to know the length of the list.
            yvalslist.append(yvals)
            if y % 6 == 5:                      ## if it's the 3rd row
                for i in xrange(int(math.floor(num_x/6.0))):## go through the 3 lists that you've made and find the average
                                                        ## values for each column so that the 6x6 square is averaged into 1
                    #print yvalslist, len(yvalslist[0]),len(yvalslist[1]),len(yvalslist[2]),(int(math.floor(num_x/3.0))+1)

                    average = yvalslist[0][i]+yvalslist[1][i]+yvalslist[2][i]+yvalslist[3][i]+yvalslist[4][i]+yvalslist[5][i]
                    temp.append(average/36)
                otherMap.append(temp)
                temp = []                       ## reset the temp list
                yvalslist = []                  ## reset the yvalslist
            num_y = y

        return otherMap, int(math.floor(num_x/6.0)), int(math.floor(num_y/6.0))




    def next(self):
        """
        This is the spot where the values are chosen and returned for moving the local map data into the hybrid

        :return:
        """
        if self._doneIter:raise StopIteration
        x,y = self._x, self._y

        if x < self._reduce_w -1:
            self._x = self._x + 1
        elif x == self._reduce_w -1:
            if y == self._reduce_h-2:
                self._doneIter = True
                self._x, self._y = 1,1
            elif y < self._reduce_h -2:
                self._x = 0
                self._y = self._y + 1

        elif x > self._reduce_w -1 or y > self._reduce_h -1:
            raise RuntimeError("Values too large")


        return ((x - self._ox),
                (y - self._oy),
                self._reducedMap[y][x])


    def __iter__(self):
        self._x, self._y = 0,0
        print "Iterating local map:", len(self._reducedMap), len(self._reducedMap[0]), self._reduce_w, self._reduce_h
        return self

    # def givePoint(self,x,y):
#(p,q) = self._map_list.lookupTransform("map","base_footprint",rospy.Time.now()) ## location of the robot in Global

