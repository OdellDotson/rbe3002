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

import logging

#Message Types
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, GridCells
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
        """ Setting up the logger first """
        logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
                    datefmt='%m-%d %H:%M',
                    filename="Whatislove.txt",
                    filemode='w')
        logger = logging.getLogger(__name__)
        logger.info("Logger started for: "+name)
        logger.info("Why is this doing this... :( ")


        logger.setLevel(logging.INFO)
        logger.info("Node created with name: "+name)
        logger.info("Creating Map Next")
        self.map = map(name+"Map")
        self._name_ = name
        logger.info("Map Created")




        logger.info("Creating Publishers and Subscribers")
        ## Pub/Sub information
        self._vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self._pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped,queue_size=1)

        self._odom_sub = rospy.Subscriber('/odom', Odometry, self.odomCallback, queue_size=3)

        self._click_sub = rospy.Subscriber('/move_base_simple/goalRBE', PoseStamped, self.storeGoal, queue_size=1) # check out the self.map.storeGoal thi
        self._map_sub = rospy.Subscriber('/map',PoseStamped, PoseStamped, self.map.broadcastLocation, queue_size=3)
        # self._bmp_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, turtlebot.setBumper, queue_size=3)


        logger.info("Creating TF Listeners and Broadcasters")
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
        logger.info("Instantiation Complete.")
        print "Robot Created"


    def storeGoal(self, msg):
        print msg
        self.grid_goalX = int(math.floor(msg.pose.position.x/0.3)) -1
        self.grid_goalY = int(math.floor(msg.pose.position.y/0.3))
        self.grid_goaltheta=tools.normalizeTheta((msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w))

        self.map.storeGoal(self.grid_goalX, self.grid_goalY, self.grid_goaltheta)
        path = self.map.getNextWaypoint()
        while path is not []:
            print path
            gx,gy = path
            self.driveTo((self.map.current_x,self.map.current_y, self.map.current_theta),
                         (gx,gy,None))
            path = self.map.getNextWaypoint()


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
        self._quatx, self._quaty, self._quatz, self._quatw = quat
        self._quat = (self._quatx, self._quaty, self._quatz, self._quatw)

    # def mapCallback(self,msg):
    #     """
    #     This will handle the map frame location of the robot.
    #
    #     :param msg: this is a message form the odom subscriber
    #     :return: None
    #     """
    #     pos,quat = self._quatFromMsg(msg)
    #     self._mapx, self._mapy, self._mapz = pos
    #     self._map_quatx, self._map_quaty, self._map_quatz, self._map_quatw = quat
    #     self._map_quat = (self._map_quatx, self._map_quaty, self._map_quatz, self._map_quatw)

    """------------------General Movement------------------"""
    """ Movement Helpers """
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
    """ Big Movement Functions """
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

        print "Driving Straight "+str(distance)+" meters"
        while ((not arrived) and (not rospy.is_shutdown())):
            ## Get the distance traveled
            dist_so_far = tools.distFormula((starting_x,starting_y),(self._x,self._y))

            ## Modulate speed based off how far you've gone
            # if dist_so_far <= abs(distance)*0.25:
            #     regulated_speed = speed*(0.2) + speed*(dist_so_far / distance)
            # elif dist_so_far >= abs(distance)*0.75:
            #     regulated_speed = speed*(0.2) + speed*(1- (dist_so_far / distance))
            # else:

            regulated_speed = speed
            if distance > 0: self._spinWheels(regulated_speed,regulated_speed,0.1)
            else: self._spinWheels(-regulated_speed,-regulated_speed,0.1)
            #Set the booleans on if you're there yet
            arrived = dist_so_far >= abs(distance)
            print arrived, dist_so_far, distance

        print "Finished Driving"
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

        print "Turning "+str(angle)+" degrees"
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

            ## If you're within 0.1 Radian stop
            if math.degrees(abs(angle_to_travel_rad) - abs(d_theta)) < 5:
                done = True
                print "Finished Turning"
                self._stopRobot()
            else:
                if (angle > 0):
                    self._spinWheels(speed,-speed,.1)
                else:
                    self._spinWheels(-speed,speed,.1)

    def driveTo(self, start, end):
        cx,cy,ct = start
        gx,gy,gt = end
        print "Start: ",start, "End: ",end

        newPose = PoseStamped()

        dx = ((gx*0.3)-cx); dy = ((gy*0.3)-cy)
        turn_theta = math.degrees(math.atan2(dy,dx))
        print "DX", dx, "Turn Theta", turn_theta

        self.rotate((turn_theta-math.degrees(ct)))
        self.driveStraight(0.1,math.sqrt((dx)**2 + (dy)**2))








    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            continue

    def main(self):
        try:
            # self.rotate(90)
            # self.driveStraight(0.5,0.5)
            while not rospy.is_shutdown():
                rospy.sleep(0.1)
                continue
            """try:
                self._odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
            except tf.Exception:
                print "Exception thrown"
                return
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



Turtle = turtlebot("HulkHogan")
Turtle.main()

