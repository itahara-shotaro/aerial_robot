#!/usr/bin/env python

import sys
import time
import rospy
import math
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import Point # /assemble_quadrotorsN/assemble/newCoG
import numpy as np

class Fig8Demo():
    def __init__(self):

        rospy.init_node("figure_8_demo")
        # state variables

        # initial CoG location (use this as the center)
        self.cog_loc_x_init = 2.0
        self.cog_loc_y_init = 1.0
        first = True
        
        # circle radius[m]
        self.circle_radius = 0.5

        # step width and counts
        self.circle_steps = 100

        # subscribers & publishers
        self.nav1_pub = rospy.Publisher("/assemble_quadrotors1/uav/nav",FlightNav , queue_size=1)
        self.nav2_pub = rospy.Publisher("/assemble_quadrotors2/uav/nav",FlightNav , queue_size=1)

        self.initial_cog_sub = rospy.Subscriber('/assemble_quadrotors1/assemble/newCoG',Point,self.initialCoGCallback)

    def initialCoGCallback(self, msg):
        if first is True:
            self.cog_loc_x_init = msg.x
            self.cog_loc_y_init = msg.y
            first = False
        else:
            return
    
    #main func
    def main(self):
        r = rospy.Rate(1) # 1hz -> angle_step[s] to reach max bend

        i=0
        rospy.sleep(1)
        print("start")
        while (not rospy.is_shutdown()) and (i <= 2*self.circle_steps):

            if i<self.circle_steps:
                
                print(f"step {i+1} of {self.circle_steps}")
                theta = i*(2*math.pi)/self.circle_steps + math.pi

                dx = self.circle_radius * math.cos(theta)
                dy = self.circle_radius * math.sin(theta)
                # prepare msgs
                updated_position = FlightNav()

                # set values
                updated_position.target = 1
                updated_position.control_frame = 0
                updated_position.pos_xy_nav_mode=2
                updated_position.target_pos_x = self.cog_loc_x_init + dx + self.circle_radius
                updated_position.target_pos_y = self.cog_loc_y_init + dy


                # send msgs
                self.nav1_pub.publish(updated_position)
                self.nav2_pub.publish(updated_position)
            
            else:
                print(f"step {i+1} of {self.circle_steps}")
                theta = -(i-self.circle_steps)*(2*math.pi)/self.circle_steps

                dx = self.circle_radius * math.cos(theta)
                dy = self.circle_radius * math.sin(theta)
                # prepare msgs
                updated_position = FlightNav()

                # set values
                updated_position.target = 1
                updated_position.control_frame = 0
                updated_position.pos_xy_nav_mode=2
                updated_position.target_pos_x = self.cog_loc_x_init + dx - self.circle_radius
                updated_position.target_pos_y = self.cog_loc_y_init + dy


                # send msgs
                self.nav1_pub.publish(updated_position)
                self.nav2_pub.publish(updated_position)
            r.sleep()
            i+=1
        print("task finished")

if __name__=="__main__":
    try:
        fig8_demo = Fig8Demo()
        fig8_demo.main()
    except rospy.ROSInterruptException: 
        pass
