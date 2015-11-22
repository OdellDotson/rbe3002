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




    def storeCostmap(self, msg):
        self._map = tools.lMaptoLLMap(msg.data,
                                      msg.info.height,
                                      msg.info.width)
        (p,q) = self._map_tfListener.lookupTransform("base_laser_link","map",rospy.Time.now())
        print "Result is: ",p,q



    # def givePoint(self,x,y):
