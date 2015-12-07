__author__ = 'Troy Hughes'


##Imports

# Simple Imports
import time

import rospy
import tf
# import tools
import math


# Subclass Information
# from gMap import gMap

import logging

#Message Types
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, GridCells, OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult


class turtlebot():
    def __init__(self, name, wheelbase=0.352):
        """
        :param name: <string> that will be the 'name'.log file and the name of the node created
        :return: None
        """


        """ SETUP::
            1) will instantiate the base logging class and start the node
            2) sets up all the publishers and subscribers
            3) ...
        """
        ## Loggers and node information
        rospy.init_node(name)

        # self.map = gMap(name+"Map")
        self._name_ = name

        self._x_offset = None
        self._y_offset = None
        self._x, self._y = 0,0


        ## Pub/Sub information
        self._drivePublisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self._cmdVelPub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)

        self._baseResultSubscriber = rospy.Subscriber('/move_base/result',MoveBaseActionResult,self._storeMoveBaseResult, queue_size=1)

        # self._odom_sub = rospy.Subscriber('/odom', Odometry, self.odomCallback, queue_size=3)
        # self._click_sub = rospy.Subscriber('/move_base_simple/goalRBE', PoseStamped, self.storeGoal, queue_size=1) # check out the self.map.storeGoal thing
        # self._local_cost = rospy.Subscriber('/move_base/local_costmap/costmap',OccupancyGrid,self.storeCostmap, queue_size=1)
        # self._map_sub = rospy.Subscriber('/map', OccupancyGrid, self._updateMap, queue_size=1)


        #### Private variables that get updated by callbacks.
        ## Odom Handlers ##
        # self._odom_list = tf.TransformListener()
        # self._odom_tf = tf.TransformBroadcaster()
        # self._odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")
        # ## Map Handlers ##
        # self._map_list = tf.TransformListener()
        # ## Goal Handlers ##
        # self._goal_list = tf.TransformListener()
        #
        # #### Private robot Information
        self._wheelbase = wheelbase
        self._startupSpinVel = None

        ## After creation, wait 1 second in order to ensure that your
        self.sleeper = rospy.Duration(1)
        rospy.sleep(self.sleeper)

        print '\n\n'
        print "Robot Created"
        print '\n\n'


    def storeGoal(self,msg):
        raise NotImplementedError("Final Project version of 'storeGoal' not implemented yet")


    def storeCostmap(self, msg):
        raise NotImplementedError("Final Project version of 'storeCostmap' not implemetned yet")


    def _updateMap(self, msg):
        raise NotImplementedError("Final Project version of '_updateMap' not implemented yet")


    def odomCallback(self, msg):
        raise NotImplementedError("Final project version of 'odomCallback' not implemented yet")




    """------------------Result Functions------------------"""
    def _storeMoveBaseResult(self, msg):
        result = msg.status.status
        self._nextAction(result,msg)

    def _nextAction(self, result,msg):
        if result == 0:
            print msg
            raise NotImplementedError("The Result == 0 occured and a response is not implimented")
        elif result == 1:
            print "Robot is moving towards it's destination"
        elif result == 2:
            raise NotImplementedError("The result == 2 occured and a response is not implemented")
        elif result == 3:
            print "Robot has arrived at it's destination"
        elif result == 4:
            print "The Robot cannot reach the position you wish to go to, please try again"
        else:
            raise RuntimeError("A case we did not think of has occured, fuck that.")



    """------------------General Movement------------------"""
    def driveTo(self,x,y,theta):
        """
        This will publish a PoseStampped message to tell the robot to move to the x,y,theta position on the map
        :param x:
        :param y:
        :param theta:
        :return:None
        """
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y

        if theta == None:
            msg.pose.orientation.x = 0
            msg.pose.orientation.y = 0
            msg.pose.orientation.z = 0
            msg.pose.orientation.w = 1
        else:
            raise NotImplementedError("Non-[0,0,0,1] pose's are not implemented")

        self._drivePublisher.publish(msg)

    def startupSpin(self, timeToSpin):
        """
        This will be the method that causes the robot to spin about it's axis for 'timeToSpin' seconds. this will
        stop the robot when it is done spinning for timeToSpin seconds.

        :param timeToSpin: <int> the length of time for the robot to spin in a circle
        :return: None
        """
        u1 = self._startupSpinVel
        u2 = -self._startupSpinVel

        lin_vel = (0.5)*(u1 + u2)
        ang_vel = (1/(self._wheelbase))*(u1-u2)

        start = time.time()
        while ((time.time() - start) < timeToSpin and (not rospy.is_shutdown())):
            self._pubTwist(lin_vel, ang_vel)
        ## stop Robot
        self._pubTwist(0,0)
        return



    def _pubTwist(self,u,w):
        """
        This is the method that publishes twist messages to the robot if you wish to use your own move commands.
        :param u: Linear Velocity
        :param w: Angular Velocity
        :return: the message that is published
        """
        twist = Twist()
        twist.linear.x = u; twist.angular.z = w

        twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0

        self._cmdVelPub.publish(twist)
        return twist



    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            print "tick"
            continue

    def main(self):
        try:
            self.driveTo(2,2,None)
            while not (rospy.is_shutdown()):
                rospy.sleep(0.1)

        except rospy.ROSInterruptException:
            pass



Turtle = turtlebot("DeezNuts")
Turtle.main()

