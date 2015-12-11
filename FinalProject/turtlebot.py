__author__ = 'Troy Hughes'

##Imports

# Simple Imports
import time
import tools
import rospy
import tf
# import tools
import math
# Subclass Information
# from gMap import gMap

from gMap import gMap
# Message Types
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, GridCells, OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from turtleExceptions import TurtlebotException


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

        self.map = gMap(name + "Map")
        self._name_ = name

        self.frontierX, self.frontierY = None, None
        self._x_offset = None
        self._y_offset = None
        self._x, self._y = 0, 0
        self._notDoneExploring = False;

        ## Pub/Sub information
        self._drivePublisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self._cmdVelPub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)

        self._baseResultSubscriber = rospy.Subscriber('/move_base/result', MoveBaseActionResult,
                                                      self._storeMoveBaseResult, queue_size=1)
        self._mapSubscriber = rospy.Subscriber('/map',OccupancyGrid, self._updateMap, queue_size=1)

        # #### Private robot Information
        self._wheelbase = wheelbase
        self._startupSpinVel = 0.5
        self._moveError = False
        self._moving = False

        ## After creation, wait 1 second in order to ensure that your
        self.sleeper = rospy.Duration(1)
        rospy.sleep(self.sleeper)

        print '\n'
        print "Robot Created"
        print '\n'

    def storeGoal(self, msg):
        raise NotImplementedError("Final Project version of 'storeGoal' not implemented yet")

    def storeCostmap(self, msg):
        raise NotImplementedError("Final Project version of 'storeCostmap' not implemetned yet")

    def _updateMap(self, msg):
        print "Map received"
        self.map.updateMap(msg)

    def odomCallback(self, msg):
        raise NotImplementedError("Final project version of 'odomCallback' not implemented yet")

    """------------------Result Functions------------------"""

    def _storeMoveBaseResult(self, msg):
        result = msg.status.status
        self._nextAction(result, msg)

    def _nextAction(self, result, msg):
        if result == 0:
            print msg
            raise NotImplementedError("The Result == 0 occured and a response is not implimented")
        elif result == 1:
            print "Robot is moving towards it's destination"
        elif result == 2:
            raise NotImplementedError("The result == 2 occured and a response is not implemented")
        elif result == 3:
            print "Robot has arrived at it's destination"
            self._moving = False
        elif result == 4:
            print "The Robot cannot reach the position you wish to go to, please try again"
            self._moving = False
            self._moveError = True
        else:
            raise RuntimeError("A case we did not think of has occured, fuck that.")

    """------------------Frontier Functions ---------------"""

    def findFrontier(self, verbose = False):
        location = self.map.getNextFrontier(verbose)
        self.frontierX, self.frontierY = location

    """------------------General Movement------------------"""

    def driveTo(self, x, y, theta):
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

        if theta is None:
            quat = (0,0,0,1)
        else:
            try:
                quat = tf.transformations.quaternion_from_euler(0.0,0.0,float(theta))
            except Exception,e:
                print "Theta is: ",theta
                raise e

        qx,qy,qz,qw = quat
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self._drivePublisher.publish(msg)
        self._moving = True

    def startupSpin(self):
        """
        This will be the method that causes the robot to spin about it's axis for 'timeToSpin' seconds. this will
        stop the robot when it is done spinning for timeToSpin seconds.

        :param timeToSpin: <int> the length of time for the robot to spin in a circle
        :return: None
        """
        print "Starting startup spin "
        # orthoSpins = [math.pi/2, math.pi, -math.pi/2, -math.pi, 0]
        for i,angle in enumerate([math.pi/2,-math.pi/2]):

            self.driveTo(tools.degmapifyValue(self.map.current_x),
                         tools.degmapifyValue(self.map.current_y),
                         angle)

            while self._moving and not self._moveError and not (rospy.is_shutdown()):
                rospy.sleep(0.1)
            if self._moveError:
                print "Caught move error in startupSpin"
                self._moveError = False
            self._moveing = False

        self.driveTo(self.map.current_x,self.map.current_y,None)
        while self._moving and not self._moveError and not (rospy.is_shutdown()):
                rospy.sleep(0.1)
        if self._moveError:
            print "Failed to return to home spot, but it's cool"
            self._moveError = False
        self.moving = False


    def _recover(self):
        """
        This is the recovery mode for the robot. This function will handle when there is a move error.

        This function should ensure that there are frontiers that can be found, and then move forward.

        :return:
        """
        self.startupSpin(30)
        try:
            self.findFrontier()
            self._moveError = False
        except Exception, e:
            print "You got an exception"
            print e
        return





    def main(self):
        print "Starting Main"
        while not self.map.doneSetup() and not (rospy.is_shutdown()):
            rospy.sleep(0.1)
        print "System is prepaired to start running"

        try:

            self._notDoneExploring = True
            self.startupSpin()
            print "This map is difficult, let me think..."
            rospy.sleep(15)
            print "Ok, I think I'm ready to try and drive somewhere."
            self.findFrontier(verbose = False)
            self.driveTo(self.frontierX,self.frontierY,None)
            while self._moving and not(rospy.is_shutdown()):
                rospy.sleep(0.1)
                if self._moveError:                                     ## This will happen whenever the robot has a goal that it decides it cannot make it to.
                    print "Your frontier location is: ", self.frontierX, self.frontierY,\
                        "You are currently at: ",tools.degmapifyValue(self.map.current_x), tools.degmapifyValue(self.map.current_y)


                    raise TurtlebotException("I'm in recovery mode! Halp me!")


            # while self._notDoneExploring and not (rospy.is_shutdown()):
            #     self.findFrontier()
            #     self.driveTo(self.frontierX,self.frontierY,None)
            #     while self._moving and not(rospy.is_shutdown()):
            #         rospy.sleep(0.1)
            #         if self._moveError:                                     ## This will happen whenever the robot has a goal that it decides it cannot make it to.
            #             self._recover()
            #
            #
            # print "Frontiers Explored,"

        except rospy.ROSInterruptException:
            pass
        print "Terminating"

Turtle = turtlebot("DeezNuts")
Turtle.main()
