#!/usr/bin/env python

import sys
import time
import rospy
import math
from std_msgs.msg import Float64
import numpy as np

class AllYaw():
    def __init__(self):

        rospy.init_node("allyaw")

        # subscribers & publishers
        self.yaw1_pub = rospy.Publisher("/target_yaw1",Float64 , queue_size=1)
        self.yaw2_pub = rospy.Publisher("/target_yaw2",Float64 , queue_size=1)

    def turn(self, angle):

        # prepare message
        msg_yaw1 = Float64()
        msg_yaw2 = Float64()

        msg_yaw1.data = angle
        msg_yaw2.data = angle
        
        # publish
        self.yaw1_pub.publish(msg_yaw1)
        self.yaw2_pub.publish(msg_yaw2)

if __name__=="__main__":
    total_args = len(sys.argv)
    try:
        if(total_args > 1):
            print(float(sys.argv[1]))
            allyaw = AllYaw()
            rospy.sleep(1)
            allyaw.turn(float(sys.argv[1]))
        else:
            print("no target yaw provided")
    except rospy.ROSInterruptException: 
        pass
