__author__ = 'Troy Hughes'

import rospy
import tf
import tools
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


    def storeCostmap(self, msg):
        try:
            self._map_tfListener.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(0.2))
        except tf.Exception:
            print "TF library chunkin out broski"
            return
        self._mapLL = tools.lMaptoLLMap(msg.data,
                                      msg.info.height,
                                      msg.info.width)
        self._mapL = msg.data

        self._max_w = 37#msg.info.width
        self._max_h = 37#msg.info.height
        self._pose = msg.info.origin.position
        try:
            (p,q) = self._map_tfListener.lookupTransform("map","base_footprint",rospy.Time.now())
        except tf.Exception:
            print "Python can't read your future, calm down"
            return
        self._ox,self._oy,_ = p


    def next(self):
        """
        This is the spot where the values are chosen and returned for moving the local map data into the hybrid

        :return:
        """
        if self._doneIter:raise StopIteration
        x,y = self._x, self._y
        print "Running the iterator for values:",x,y," Max values:",self._max_h, self._max_w, "offset: ", self._ox, self._oy

        self._oxgrid = self._ox / 0.05 # TODO: Make this read in from the actual message, I think the resolution is there
        self._oygrid = self._oy / 0.05 # TODO: somewhere, but I'm not sure where.
        if x < self._max_w -1:
            self._x = self._x + 1
        elif x == self._max_w -1:
            if self._y == self._max_h-1:
                self._doneIter = True
                self._x, self._y = 0,0
            elif self._y < self._max_h -1:
                self._x = 0
                self._y = self._y + 1

        elif x > self._max_w -1 or y > self._max_h -1:
            raise RuntimeError("Values too large")

        print self._max_h -1 -43
        return (x,y,self._mapLL[y - int(self._oygrid)][x - int(self._oxgrid)]) # TODO: I don't actually know what to be adding or subtracting here. Fix pls


    def __iter__(self):
        self._x, self._y = 0,0
        print "Iterating local map:"
        return self

    # def givePoint(self,x,y):
#(p,q) = self._map_list.lookupTransform("map","base_footprint",rospy.Time.now()) ## location of the robot in Global

