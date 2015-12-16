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
        :param wheelbase: <float> that will be the width of the robot
        :return: None
        """

        """ SETUP::
            1) Will initialize ROS node under name passed
            2) Will initialize subclass information
            3) Will initialize localization variables
            4) Will Create Publishers and Subscribers in the class
            5) Will initialize specific robot variables
        """
        ## Loggers and node information
        rospy.init_node(name)

        ## Subclass Functions
        self.map = gMap(name + "Map","/GOAL")
        self._name_ = name

        ## Localization Variables
        self.frontierX, self.frontierY = None, None
        self._x_offset = None
        self._y_offset = None
        self._x, self._y = 0, 0
        self._notDoneExploring = False

        ## Pub/Sub information
        self._drivePublisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self._cmdVelPub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)

        self._baseResultSubscriber = rospy.Subscriber('/move_base/result', MoveBaseActionResult,self._storeMoveBaseResult, queue_size=1)
        self._mapSubscriber = rospy.Subscriber('/map',OccupancyGrid, self._updateMap, queue_size=1)

        # #### Private robot Information
        self._wheelbase = wheelbase
        self._startupSpinVel = 0.5
        self._moveError = False
        self._moving = False
        self._failedList = []
        self._recovery = 0

        ## After creation, wait 1 second in order to ensure that your
        self.sleeper = rospy.Duration(1)
        rospy.sleep(self.sleeper)

        print '\n'
        print "Robot Created"
        print '\n'


    """ ---------------------------------------------------------------------------------------
        Active Callbacks :: Necessary for functionality
    --------------------------------------------------------------------------------------- """
    ## '/map' Callback
    def _updateMap(self, msg):
        self.map.updateMap(msg)

    ## '/move_base/result' Callback
    def _storeMoveBaseResult(self, msg):
        result = msg.status.status
        self._nextAction(result, msg)

    ## Action Status Handler
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
            self._failedList = []
            self._moving = False
        elif result == 4:
            print "The Robot cannot reach the position you wish to go to, please try again"
            self._failedList.append((self.frontierX,self.frontierY))
            self._moving = False
            self._moveError = True
        else:
            raise RuntimeError("A case we did not think of has occured, fuck that.")

    """ ---------------------------------------------------------------------------------------
        Frontier Functions : Runs map frontier location functions, set's the frontier
        locaitons in the turtlebot scope so that the robot can know where to go. This value
        is returned to be in the global scope.
    --------------------------------------------------------------------------------------- """

    def findFrontier(self, verbose = False):
        location = self.map.getNextFrontier(verbose)
        self.frontierX, self.frontierY,termination = location
        if termination:
            raise TurtlebotException("You are done exploring, your map is complete")

    """ ---------------------------------------------------------------------------------------
        General Movement : These functions are the movement functions for the turtlebot.
        All of the following functions should be in the units that the map being read from
        This means:
            Read from /map then the unit's of move should be in /map.
    --------------------------------------------------------------------------------------- """

    def driveTo(self, x, y, theta):
        """
        This will publish a PoseStampped message to tell the robot to move to the x,y,theta position on the map
        This function will also publish where it wasnts to go to the /GOAL topic AND will set the self._moving
        value to true, so the system knows if the robot is moving or not

        :param x: global x value
        :param y: global y value
        :param theta: Theta the robot should look in
        :return:None
        """
        if x is None or y is None:
            raise TurtlebotException("It's none bruh. Why do you want me to drive to none? ")
        print "Robot will now drive to: ",x,y
        self.map.paintGoal((x,y),True)

        ### Create the poseStamped message to publish ###
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

        ### Publish the message and let the robot know it's moving ###
        self._drivePublisher.publish(msg)
        self._moving = True

    def startupSpin(self, sleepTime = 3, verbose = False):
        """
        This method causes the robot to try and randomly move about the field to rapidly learn about it's environment.
        :param sleepTime: <int> The duration that the system will sleep in between locations
        :param verbose : <boolean> if the robot should speak to us
        :return: None
        """
        start = self.map.getRobotPosition()
        startX,startY = start

        print "Running 'startupSpin' algorithm."
        locations = [(startX-1,startY),
                     (startX,startY-1),
                     (startX+1,startY),
                     (startX,startY+1),
                     (startX, startY)]
        for point in locations:
            x,y = point
            self.driveTo(x,y,None)
            while self._moving and not self._moveError and not (rospy.is_shutdown()):
                rospy.sleep(0.1)
            if self._moveError:
                if verbose: print "Caught move error in startupSpin"
                self._moveError = False
            self._moveing = False
            rospy.sleep(sleepTime)

        if verbose: print "Exiting 'startupSpin' algorithm"


    def _recover(self):
        """
        This is the recovery mode for the robot. This function will handle when there is a move error.

        This function should ensure that there are frontiers that can be found, and then move forward.

        :return:
        """
        print "\t ------- Recovery Mode ---------"
        rospy.sleep(0.5)
        self.startupSpin(sleepTime=1)
        try:
            self.findFrontier()
            self._moveError = False
        except Exception, e:
            print "You got an exception"
            print e
        return

    def _recoverDuh(self):
        """
        This is the recovery mode for the robot. This function will handle when there is a move error.

        This function should ensure that there are frontiers that can be found, and then move forward.

        :return:
        """
        print "\t ------- Recovery Mode ---------"
        rospy.sleep(0.5)
        self.startupSpin(sleepTime=1)
        try:
            self._moveError = False
        except Exception, e:
            print "You got an exception"
            print e
        return


    def testFrontierFinding(self,cacheName = 'FrontierFiles.txt'):
        """
        This function is used to test if a frontier finder is working as well as the painting of the
        frontiers. To use this, there must be a file of name <cacheName> and that file must be formatted
        as a list of list of touples;. This will be used as the frontiers and the system will display
        the frontiers accordingly.
        :return:
        """
        print "Reading frontiers"
        frontierList = eval(self.map.FE.virtualGetFrontierList(cacheName))

        print "There are "+str(len(frontierList[0]))+" Frontiers in your list"
        gridFrontierList = []
        for frontier in frontierList:
            if len(frontier) > 2:
                someFrontier = []
                for point in frontier:
                    x,y = point
                    someFrontier.append(self.map.convertMapToGlobal((x,y)))
                gridFrontierList.append(someFrontier)

        print "Painting Frontiers"
        for i,frontier in enumerate(gridFrontierList):
            print "This is the "+str(i)+" frontier"
            for j in xrange(40):
                self.map.painter.paint('/GOAL',frontier)
            rospy.sleep(2)
            print "Here comes the next frontier"
            print ""
        print "Done Painting"
        rospy.spin()


    def main(self):
        print "Starting Main"
        while not self.map.doneSetup() and not (rospy.is_shutdown()):
            rospy.sleep(0.1)
        print "System is prepaired to start running"

        try:

            self._notDoneExploring = True
            self.startupSpin()
            print "This map is difficult, let me think..."
            rospy.sleep(5)
            print "Ok, I think I'm ready to try and drive somewhere."
            self._failedList = []
            # self.findFrontier(verbose = False)
            #
            # self.driveTo(self.frontierX,self.frontierY,None)
            while self._notDoneExploring and not (rospy.is_shutdown()):
                self.findFrontier()
                self.driveTo(self.frontierX,self.frontierY,None)

                ## Wait until the robot is either: A) done moving or B) it cannot reach the location it wants to go to
                while self._moving and (not self._moveError) and not(rospy.is_shutdown()):
                    rospy.sleep(0.1)

                ## If the robot quit the loop because there's a move error:
                if self._moveError:
                    try:                                ## Try to recover
                        self._recover()
                    except TurtlebotException,e:        ## here for detecting that you finished
                        print e
                        self._notDoneExploring = False

            print "Frontiers Explored,"

        except rospy.ROSInterruptException:
            pass
        print "Terminating"

    def auxillary(self):
        print "Starting auxillary"
        while not self.map.doneSetup() and not (rospy.is_shutdown()):
            rospy.sleep(0.1)
        print "System is prepaired to start running"

        try:

            self._notDoneExploring = True
            self.startupSpin()
            print "This map is difficult, let me think..."
            rospy.sleep(5)
            print "Ok, I think I'm ready to try and drive somewhere."

            counter = 0

            while self._notDoneExploring and not (rospy.is_shutdown()):
                fq = self.map.getNavFrontiers(verbose = True)


                while not fq.empty() and not rospy.is_shutdown():
                    queueItem = fq.get()
                    p,frontierInfo = queueItem
                    frontierPoint,frontier = frontierInfo
                    self.map.paintFrontier(frontier)
                    drivePoint = self.map.convertMapToGlobal(frontierPoint)
                    dx,dy = drivePoint

                    self.driveTo(dx,dy, None)

                    while self._moving and (not self._moveError) and not(rospy.is_shutdown()):
                        rospy.sleep(0.1)

                    ## If the robot quit the loop because there's a move error:
                    # if self._moveError:
                    #     self._moveError = False
                    #     try:
                    #         path = self.map.PP._getPath(self.map._map,
                    #                                     (self.map.current_x,self.map.current_y),
                    #                                     (dx,dy))
                    #         travelPoint = self.map.PP.frontierToTravelPoint(self.map._map,
                    #                                                     (self.map.current_x,self.map.current_y),
                    #                                                     (dx,dy))
                    #         tx,ty = travelPoint
                    #         self.driveTo(tx,ty, None)
                    #         while self._moving and (not self._moveError) and not(rospy.is_shutdown()):
                    #             rospy.sleep(0.1)
                    #     except Exception,e:
                    #         print "Something went wrong and IDK what..."
                    #         print e
                    if self._moveError:
                        self._moveError = False
                        continue

                    break
                if not fq.empty():
                    counter = 0

                if fq.empty():
                    self._recoverDuh()
                    counter = counter +1

                if fq.empty() and counter == 2:
                    self._notDoneExploring = False
                rospy.sleep(4)

            print "Frontiers Explored,"

        except rospy.ROSInterruptException:
            pass
        print "Terminating"




Turtle = turtlebot("Spanish Inquisition")
Turtle.auxillary()
