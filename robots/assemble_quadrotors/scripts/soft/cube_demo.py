#!/usr/bin/env python

import sys
import time
import rospy
import math
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import Point # /assemble_quadrotorsN/assemble/newCoG
import numpy as np

class CubeDemo():
    def __init__(self):

        rospy.init_node("cube_demo")
        # state variables

        # initial CoG location (use this as the center)
        self.cog_loc_x_init = 2.0
        self.cog_loc_y_init = 1.0
        self.cog_loc_z_init = 0.5
        self.first = True
        
        # cube length[m]
        self.cube_length = 0.5

        # offset list
        self.cube_offset = [[self.cube_length, 0.0,              0.0],
                            [self.cube_length, 0.0,              self.cube_length],
                            [self.cube_length, self.cube_length, self.cube_length],
                            [self.cube_length, self.cube_length, 0.0],
                            [0.0,              self.cube_length, 0.0],
                            [0.0,              self.cube_length, self.cube_length],
                            [0.0,              0.0,              self.cube_length],
                            [0.0,              0.0,              0.0]]

        self.move_time = 1.0

        # subscribers & publishers
        self.nav1_pub = rospy.Publisher("/assemble_quadrotors1/uav/nav",FlightNav , queue_size=1)
        self.nav2_pub = rospy.Publisher("/assemble_quadrotors2/uav/nav",FlightNav , queue_size=1)

        self.initial_cog_sub = rospy.Subscriber('/assemble_quadrotors1/assemble/newCoG',Point,self.initialCoGCallback)

    def initialCoGCallback(self, msg):
        if self.first is True:
            self.cog_loc_x_init = msg.x
            self.cog_loc_y_init = msg.y
            self.first = False
        else:
            return
    
    #main func
    def main(self):
        r = rospy.Rate(1.0/self.move_time) # 0.2hz (1 every 5sec)

        i=0
        rospy.sleep(1)
        print("start")
        while (not rospy.is_shutdown()) and (i < 8):
    
            print(f"step {i+1} of 8")

            # prepare msgs
            updated_position = FlightNav()

            # set values
            updated_position.target = 1
            updated_position.control_frame = 0
            updated_position.pos_xy_nav_mode=2
            updated_position.pos_z_nav_mode=2
            updated_position.target_pos_x = self.cog_loc_x_init + self.cube_offset[i][0]
            updated_position.target_pos_y = self.cog_loc_y_init + self.cube_offset[i][1]
            updated_position.target_pos_z = self.cog_loc_z_init + self.cube_offset[i][2]

            # send msgs
            self.nav1_pub.publish(updated_position)
            self.nav2_pub.publish(updated_position)
            
            r.sleep()
            i+=1
        print("task finished")

if __name__=="__main__":
    try:
        cube_demo = CubeDemo()
        cube_demo.main()
    except rospy.ROSInterruptException: 
        pass
