__author__ = 'Troy Hughes'


##Imports

# Simple Imports
import time

import rospy
import tf
import tools
import math


# Subclass Information
from communicator import communicator
from map import map
from localMap import localMap

import logging

#Message Types
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, GridCells, OccupancyGrid
from geometry_msgs.msg import PoseStamped

class turtlebot(communicator):
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

        self.map = map(name+"Map")
        self.local = localMap(name+'Local Map')
        self._name_ = name

        self._x_offset = 0
        self._y_offset = 0


        ## Pub/Sub information
        self._vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        # self._mov_pub = rospy.Publisher('/move_base/goal',PoseStamped,queue_size=1)


        self._odom_sub = rospy.Subscriber('/odom', Odometry, self.odomCallback, queue_size=3)
        self._click_sub = rospy.Subscriber('/move_base_simple/goalRBE', PoseStamped, self.storeGoal, queue_size=1) # check out the self.map.storeGoal thing
        self._local_cost = rospy.Subscriber('/move_base/local_costmap/costmap',OccupancyGrid,self.storeCostmap, queue_size=1)
        # self._map_sub = rospy.Subscriber('/map',PoseStamped,self.mapCallback, queue_size=3)
        # self._bmp_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, turtlebot.setBumper, queue_size=3)


        #### Private variables that get updated by callbacks.
        ## Odom Handlers ##
        self._odom_list = tf.TransformListener()
        self._odom_tf = tf.TransformBroadcaster()
        self._odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")
        ## Map Handlers ##
        self._map_list = tf.TransformListener()
        ## Goal Handlers ##
        self._goal_list = tf.TransformListener()

        #### Private robot Information
        self._wheelbase = wheelbase

        ## After creation, wait 1 second in order to ensure that your
        self.sleeper = rospy.Duration(1)
        rospy.sleep(self.sleeper)

        print "Robot Created"


	

    def storeGoal(self,msg):
        # goalX = tools.mapifyValue(msg.pose.position.x) -1
        # goalY = tools.mapifyValue(msg.pose.position.y)
        goalX = tools.gmapifyValue(msg.pose.position.x)
        goalY = tools.gmapifyValue(msg.pose.position.y)
        goaltheta=tools.normalizeTheta((msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w))

        self._goal_,self._current_ = self.map.storeGoal(goalX,goalY,goaltheta)

        try:
            path = self.map.getNextWaypoint()
            print path
        except RuntimeError,e:
            print e
        return



    def storeCostmap(self, msg):
        self.local.storeCostmap(msg)
        for i in self.local:
            x,y,val = i
            self.map.addValue(x,y,val)
        self.map.repaint()



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
        This will handle the odom frame location and information

        :param msg: this is a message form the odom subscriber
        :return: None
        """
        pos,quat = self._quatFromMsg(msg)
        self._x, self._y, self._z = pos
        self._x = self._x + self._x_offset
        self._y = self._y + self._y_offset
        self._quatx, self._quaty, self._quatz, self._quatw = quat
        self._quat = (self._quatx, self._quaty, self._quatz, self._quatw)

        self._theta = tools.normalizeTheta(self._quat)


    """------------------General Movement------------------"""
    """ Movement Helpers """
    def _stopRobot(self):
        """  Publishes a stop message to the robot
        :return:None
        """
        self._publishTwist(0,0)

    # this function creates the twist message and sends it at 10Hz for a given time
    def setVel(self, lVel, aVel, time):
        # Initialize the twist message and all it's attributes
        twist = Twist()
        twist.linear.x = lVel
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = aVel

        # calculate how many times to loop
        n = time*10;

        # publish the message at 10Hz
        while not rospy.is_shutdown() and n > 0:
            n -= 1

            pub.publish(twist)

            rospy.sleep(0.1)

        stop()

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



    """ Big Movement Functions """
    def driveStraight(self, speed, distance):
        """
        This takes in a speed and distance and moves in the facing direction
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
                regulated_speed = speed*(0.2) + speed*(dist_so_far / distance)
            elif dist_so_far >= abs(distance)*0.75:
                regulated_speed = speed*(0.2) + speed*(1- (dist_so_far / distance))
            else:
                regulated_speed = speed
            if distance > 0: self._spinWheels(regulated_speed,regulated_speed,0.1)
            else: self._spinWheels(-regulated_speed,-regulated_speed,0.1)

            #Set the booleans on if you're there yet
            arrived = dist_so_far >= abs(distance)

        self._stopRobot()


    def rotate(self, angle, speed = 0.05):
        """
        :param angle: <int or double> the angle the robot should turn in degrees
        :param speed: <int or double> The speed at which the robot should rotate
        :return: None
        """
        ## Error Checking Angle:
        if angle == 0: return
        else: done = False

        angle_to_travel_rad = math.radians(angle)
        start_theta_rad = tools.normalizeTheta(self._quat)
        print angle_to_travel_rad, start_theta_rad
        print ("angle to travel", "start theta", "Current theta", "d_theta")

        while not done and (not rospy.is_shutdown()):
            current_theta_rad = tools.normalizeTheta(self._quat)

            # Calculates the difference in the thetas
            # This keeps track of
            debug_state = None
            if start_theta_rad > current_theta_rad and start_theta_rad > math.pi*3/2 and current_theta_rad < math.pi/2:
                d_theta = ((2*math.pi) - start_theta_rad) + current_theta_rad
                debug_state = 1
            elif start_theta_rad < current_theta_rad and start_theta_rad < math.pi/2 and current_theta_rad > math.pi*3/2:
                d_theta = ((2*math.pi) - current_theta_rad) + start_theta_rad
                debug_state = 2
            else:
                d_theta = start_theta_rad - current_theta_rad
                debug_state = 3

            print angle_to_travel_rad, start_theta_rad, current_theta_rad, d_theta, debug_state
            ## If you're within 0.1 Radian stop
            if abs(angle_to_travel_rad) - abs(d_theta) < 0.05:
                done = True
                self._stopRobot()
            else:
                if (angle > 0):
                    self._spinWheels(speed,-speed,.1)
                else:
                    self._spinWheels(-speed,speed,.1)


    def driveTo(self, point):
        ## http://wiki.ros.org/move_base
        ## http://wiki.ros.org/base_local_planner
        cx,cy,ctheta = self._current_
        gx,gy,gtheta = self._goal_

        dx = tools.demapifyValue(gx - cx)
        dy = tools.demapifyValue(gy - cy)
        turn_theta = math.degrees(math.atan2(dy,dx))

        print "Rotating "+str((turn_theta - math.degrees(ctheta)))+" degrees"
        self.rotate((turn_theta - math.degrees(ctheta)))
        print "Driving "+str(math.sqrt((dx)**2 + (dy)**2))+" meters"
        self.driveStraight(0.1,math.sqrt((dx)**2 + (dy)**2))



        print "I did it!"
        return




    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            print "tick"
            continue

    def main(self):
        try:
            self._x_offset, self._y_offset = self.map.getRobotPosition()

            self._x = self._x + self._x_offset
            self._y = self._y + self._y_offset

            #print "goal: ", self.map.getRobotPosition()
            self._theta = self.map.getRobotAngle
            self._goal_,self._current_ = self.map.storeGoal(self._x, self._y, tools.normalizeTheta(self._quat))
            while not rospy.is_shutdown():
                goalX, goalY, goalT = self._goal_
                #print "Goal position is ", goalX, goalY, goalT
                #print "Robot position is ", self._x, self._y, tools.normalizeTheta(self._quat)

                if self.map.isAtGoalPosition((self._x, self._y)):
                    #print "At goal position"
                    if not self.map.isAtGoalAngle(tools.normalizeTheta(self._quat)):
                        # rotate to goal
                        #print "Rotating to goal angle"
                        rospy.sleep(0.1)
                    else:
                        #print "At goal angle"
                        rospy.sleep(0.1)
                else:
                    #print "Navigating to next waypoint: ", self.map.getNextWaypoint()
                    rospy.sleep(1.0)
                continue
            """
            p,q = self._map_list.lookupTransform("map","base_footprint",rospy.Time(0))
            print p,q
            self.rotate(90)
            p,q = self._map_list.lookupTransform("map","base_footprint",rospy.Time(0))
            print p,q
            self.driveStraight(1,1)
            p,q = self._map_list.lookupTransform("map","base_footprint",rospy.Time(0))
            print p,q"""
        except rospy.ROSInterruptException:
            pass



Turtle = turtlebot("DeezNuts")
Turtle.main()

