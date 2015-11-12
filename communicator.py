__author__ = 'Troy Hughes'

#Message Types
from geometry_msgs.msg import Twist

class communicator(object):

    def _publishTwist(self, u, w, publisher):
        """
        This function creates a twist message to give to the publisher to publish

        :param u: linear velocity
        :param w: angular velocity
        :param publisher: The publisher to send it to the topic
        :return:
        """

        twist = Twist()
        twist.linear.x = u; twist.angular.z = w

        twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0

        publisher.publish(twist)


    def _quatFromMsg(self, msg):
        """
        :param data: Callback Message from a general movement subscriber
        :return: the msg stripped by a pose
        """

        return (msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                0), \
               (msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)


