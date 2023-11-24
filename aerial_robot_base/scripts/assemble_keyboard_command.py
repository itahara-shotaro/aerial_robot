#!/usr/bin/env python
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8
import rosgraph

import sys, select, termios, tty

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

if __name__=="__main__":
        settings = termios.tcgetattr(sys.stdin)
        rospy.init_node("keyboard_command")
        male_robot_ns = "/assemble_quadrotors1"
        female_robot_ns = "/assemble_quadrotors2"
        male_ns = male_robot_ns + "/teleop_command"
        female_ns = female_robot_ns + "/teleop_command"

        male_land_pub = rospy.Publisher(male_ns + '/land', Empty, queue_size=1)
        male_halt_pub = rospy.Publisher(male_ns + '/halt', Empty, queue_size=1)
        male_start_pub = rospy.Publisher(male_ns + '/start', Empty, queue_size=1)
        male_takeoff_pub = rospy.Publisher(male_ns + '/takeoff', Empty, queue_size=1)
        male_force_landing_pub = rospy.Publisher(male_ns + '/force_landing', Empty, queue_size=1)
        male_ctrl_mode_pub = rospy.Publisher(male_ns + '/ctrl_mode', Int8, queue_size=1)

        female_land_pub = rospy.Publisher(female_ns + '/land', Empty, queue_size=1)
        female_halt_pub = rospy.Publisher(female_ns + '/halt', Empty, queue_size=1)
        female_start_pub = rospy.Publisher(female_ns + '/start', Empty, queue_size=1)
        female_takeoff_pub = rospy.Publisher(female_ns + '/takeoff', Empty, queue_size=1)
        female_force_landing_pub = rospy.Publisher(female_ns + '/force_landing', Empty, queue_size=1)
        female_ctrl_mode_pub = rospy.Publisher(female_ns + '/ctrl_mode', Int8, queue_size=1)

        motion_start_pub = rospy.Publisher('assemble_start', Empty, queue_size=1)
        motion_stop_pub  = rospy.Publisher('assemble_stop', Empty, queue_size=1)



        #the way to write publisher in python
        ## ToDo: set different operation topics for each airframe
        comm=Int8()
        gain=UInt16()
        try:
                while(True):
                        key = getKey()
                        print("the key value is {}".format(ord(key)))
                        # takeoff and landing
                        if key == 'l':
                                male_land_pub.publish(Empty())
                                female_land_pub.publish(Empty())
                        if key == 'r':
                                male_start_pub.publish(Empty())
                                female_start_pub.publish(Empty())
                        if key == 'h':
                                male_halt_pub.publish(Empty())
                                female_halt_pub.publish(Empty())
                        if key == 'f':
                                male_force_landing_pub.publish(Empty())
                                female_force_landing_pub.publish(Empty())
                        if key == 't':
                                male_takeoff_pub.publish(Empty())
                                female_takeoff_pub.publish(Empty())
                        if key == 'v':
                                comm.data = 1
                                male_ctrl_mode_pub.publish(comm)
                                female_ctrl_mode_pub.publish(comm)
                        if key == 'p':
                                comm.data = 0
                                male_ctrl_mode_pub.publish(comm)
                                female_ctrl_mode_pub.publish(comm)
                        if key == 's':
                                motion_start_pub.publish(Empty())
                        if key == 'e':
                                motion_stop_pub.publish(Empty())
                        if key == '\x03':
                                break
                        rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)