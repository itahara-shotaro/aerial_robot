#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist
import tf2_ros as tf2

class TwistLookupNode:
    def __init__(self):
        rospy.init_node('twist_lookup_node')

        self.tl = tf.TransformListener()

        # Timer to call the lookup function every 0.5 seconds
        self.timer = rospy.Timer(rospy.Duration(0.01), self.lookup_twist)
        self.publisher = rospy.Publisher('/quadrotor2_twist', Twist, queue_size=10)

        rospy.spin()

    def lookup_twist(self, event):
        try:
            # Lookup the transformation from "world" to "quadrotor"
            tw=self.tl.lookupTwist("quadrotor1/main_body2", "world",rospy.Time(0), rospy.Duration(0.1))
            #print(tw[1][2]) # tw[0] : lienar vel, tw[1] : angular vel
            msg=Twist()
            msg.linear.x=tw[0][0]
            msg.linear.y=tw[0][1]
            msg.linear.z=tw[0][2]
            msg.angular.x=tw[1][0]
            msg.angular.y=tw[1][1]
            msg.angular.z=tw[1][2]
            self.publisher.publish(msg)


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Error during tf lookup")
        except (tf2.TransformException):
            rospy.logwarn("tf tree is not ready yet")

if __name__ == '__main__':
    try:
        TwistLookupNode()
    except rospy.ROSInterruptException:
        pass