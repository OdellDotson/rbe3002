__author__ = 'Troy Hughes'


##Imports

# Simple Imports
import time

import rospy
import tf
import tools


# Subclass Information
from log_base import log_base
from communicator import communicator

#Message Types
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class turtlebot(log_base,communicator):
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
        log_base.__init__(self,name)
        rospy.init_node(name)
        self._name_ = name

        ## Pub/Sub information
        self._vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

        self._odom_sub = rospy.Subscriber('/odom', Odometry, self.odomCallback, queue_size=3)
        # self._bmp_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, turtlebot.setBumper, queue_size=3)

        #### Private variables that get updated by callbacks.
        ## Odom Handlers ##
        self._odom_listener = tf.TransformListener()
        self._odom_tf = tf.TransformBroadcaster()
        self._odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")
        ## Map Handlers ##
        self._map_list = tf.TransformListener()
        self._map_tf = tf.TransformBroadcaster()
        ## Goal Handlers ##
        self._goal_list = tf.TransformListener()
        self._goal_tf = tf.TransformBroadcaster()

        #### Private robot Information
        self._wheelbase = wheelbase



        ## After creation, wait 1 second in order to ensure that your
        self.sleeper = rospy.Duration(1)
        rospy.sleep(self.sleeper)

    """ Overridden or SubUsed functions """
    def _publishTwist(self, u, w):
        """
        :param u: Linear Velocity
        :param w: Angular Velocity
        :return: None
        """
        communicator._publishTwist(self, u, w,self._vel_pub)
        return

    def odomCallback(self, msg):
        """
        This will unpack the quaternian style message into an x,y,z in the map
        and the quaternian x,y,z,w

        :param msg: this is a message form the odom subscriber
        :return: None
        """
        pos,quat = self._quatFromMsg(msg)
        self._x, self._y, self._z = pos
        self._quatx, self._quaty, self._quatz, self._quatw = quat


    """ General Movement """
    def _stopRobot(self):
        """  Publishes a stop message to the robot
        :return:None
        """
        self._publishTwist(0,0)

    def _spinWheels(self,u1,u2,timesec):
        """
        This will spin the left wheel u1 and the right wheel u2 for timesec <seconds>

        :param u1: left Wheel velocity
        :param u2: Right Wheel Velocity
        :param timesec: How long this will happen for
        :return: None
        """
        lin_vel = (0.5)*(u1 + u2)
        ang_vel = (1/(self._wheelbase))*(u1-u2)

        start = time.time()
        while ((time.time() - start) < timesec and (not rospy.is_shutdown())):
            self._publishTwist(lin_vel, ang_vel)
        ## stop Robot
        self._stopRobot()
        return

    def driveStraight(self, speed, distance):
        """
        THis takes in a speed and distance and moves in the facing direction
        until it has reached the distance.

        NOTE: This function denotes it's travel by how far the 'odom' frame believes
        it has traveled. NOT how far it has traveled in the /map.

        :param speed: This is a 0 to 1 value, it will cap the values at 1 and 0.
        :param distance: This is a value in meters.
        :return: Nothing
        """

        ## Speed Error checking
        if speed > 1: speed = 1
        elif speed < 0: speed = 0

        ## Set the starting x,y
        starting_x = self._x
        starting_y = self._y

        ## Booleans for understanding location
        arrived = False
        slowdown = False

        while ((not arrived) and (not rospy.is_shutdown())):
            ## Get the distance traveled
            dist_so_far = tools.distFormula((starting_x,starting_y),(self._x,self._y))

            ## Modulate speed based off how far you've gone
            if dist_so_far <= abs(distance)*0.25:
                print 'there'
                regulated_speed = speed*(0.2) + speed*(dist_so_far / distance)
            elif dist_so_far >= abs(distance)*0.75:
                print 'Here'
                regulated_speed = speed(0.2) + speed*(1- (dist_so_far / distance))
            else:
                regulated_speed = speed
            if distance > 0: self._spinWheels(regulated_speed,regulated_speed,0.1)
            else: self._spinWheels(-regulated_speed,-regulated_speed,0.1)

            #Set the booleans on if you're there yet
            arrived = dist_so_far >= abs(distance)

        self._stopRobot()


    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            continue

    def main(self):
        try:
            self.driveStraight(1,1)
        except rospy.ROSInterruptException:
            pass



Turtle = turtlebot("Test")
Turtle.main()

