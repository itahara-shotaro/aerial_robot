#!/usr/bin/env python

import sys
import time
import rospy
import math
from std_msgs.msg import Float64
import numpy as np

class BendDemo():
    def __init__(self):

        rospy.init_node("bend_demo")

        # subscribers & publishers
        self.yaw1_pub = rospy.Publisher("/target_yaw1",Float64 , queue_size=1)
        self.yaw2_pub = rospy.Publisher("/target_yaw2",Float64 , queue_size=1)

        self.pitch1_pub = rospy.Publisher("/target_pitch1",Float64 , queue_size=1)
        self.pitch2_pub = rospy.Publisher("/target_pitch2",Float64 , queue_size=1)

        # state variables

        # maximum allowed pitch/yaw (will move inside this)
        self.max_yaw = 0.5
        self.max_pitch = 0.0

        # step width and counts
        self.angle_step = 10
        self.step_count = 0


    #main func
    def main(self):
        r = rospy.Rate(0.3) # 1hz -> angle_step[s] to reach max bend

        while not rospy.is_shutdown():
            print((self.max_yaw)*math.sin((math.pi/2)*(1/self.angle_step)*self.step_count))

            # prepare msgs
            yaw1_msg = Float64()
            yaw2_msg = Float64()
            pitch1_msg = Float64()
            pitch2_msg = Float64()

            # set values
            yaw1_msg.data = (self.max_yaw/2)*math.sin((math.pi/2)*(1/self.angle_step)*self.step_count)
            yaw2_msg.data = -1*(self.max_yaw/2)*math.sin((math.pi/2)*(1/self.angle_step)*self.step_count)

            pitch1_msg.data = (self.max_pitch/2)*math.sin((math.pi/2)*(1/self.angle_step)*self.step_count)
            pitch2_msg.data = (self.max_pitch/2)*math.sin((math.pi/2)*(1/self.angle_step)*self.step_count)

            # send msgs
            self.yaw1_pub.publish(yaw1_msg)
            self.yaw2_pub.publish(yaw2_msg)
            self.pitch1_pub.publish(pitch1_msg)
            self.pitch2_pub.publish(pitch2_msg)

            r.sleep()
            self.step_count+=1
            if self.step_count%self.angle_step==0:
                print("waiting for 20 seconds")
                print(f"angle:{(self.max_yaw/2)*math.sin((math.pi/2)*(1/self.angle_step)*self.step_count)}")
                rospy.sleep(20)
            


if __name__=="__main__":
    try:
        bend_demo = BendDemo()
        bend_demo.main()
    except rospy.ROSInterruptException: 
        pass
