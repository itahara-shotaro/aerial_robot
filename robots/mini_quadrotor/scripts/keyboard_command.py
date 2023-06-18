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
        #robot_ns = rospy.get_param("~robot_ns", "");
        robot_ns1="assemble_quadrotors1"
        robot_ns2="assemble_quadrotors2"
        # robot_ns1="quadrotor1"
        # robot_ns2="quadrotor2"

        ns1 = robot_ns1 + "/teleop_command"
        land_pub1 = rospy.Publisher(ns1 + '/land', Empty, queue_size=1)
        halt_pub1 = rospy.Publisher(ns1 + '/halt', Empty, queue_size=1)
        start_pub1 = rospy.Publisher(ns1 + '/start', Empty, queue_size=1)
        takeoff_pub1 = rospy.Publisher(ns1 + '/takeoff', Empty, queue_size=1)
        force_landing_pub1 = rospy.Publisher(ns1 + '/force_landing', Empty, queue_size=1)
        ctrl_mode_pub1 = rospy.Publisher(ns1 + '/ctrl_mode', Int8, queue_size=1)
        motion_start_pub1 = rospy.Publisher('task_start', Empty, queue_size=1)

        ns2 = robot_ns2 + "/teleop_command"
        land_pub2 = rospy.Publisher(ns2 + '/land', Empty, queue_size=1)
        halt_pub2 = rospy.Publisher(ns2 + '/halt', Empty, queue_size=1)
        start_pub2 = rospy.Publisher(ns2 + '/start', Empty, queue_size=1)
        takeoff_pub2 = rospy.Publisher(ns2 + '/takeoff', Empty, queue_size=1)
        force_landing_pub2 = rospy.Publisher(ns2 + '/force_landing', Empty, queue_size=1)
        ctrl_mode_pub2 = rospy.Publisher(ns2 + '/ctrl_mode', Int8, queue_size=1)
        motion_start_pub2 = rospy.Publisher('task_start', Empty, queue_size=1)


        comm=Int8()
        gain=UInt16()
        try:
                while(True):
                        key = getKey()
                        print("the key value is {}".format(ord(key)))
                        # takeoff and landing
                        if key == 'l':
                                land_pub1.publish(Empty())
                                land_pub2.publish(Empty())
                                #for hydra joints
                        if key == 'r':
                                start_pub1.publish(Empty())
                                start_pub2.publish(Empty())
                                #for hydra joints
                        if key == 'h':
                                halt_pub1.publish(Empty())
                                halt_pub2.publish(Empty())
                                 #for hydra joints
                        if key == 'f':
                                force_landing_pub1.publish(Empty())
                                force_landing_pub2.publish(Empty())
                        if key == 't':
                                takeoff_pub1.publish(Empty())
                                takeoff_pub2.publish(Empty())
                        if key == 'u':
                                stair_pub.publish(Empty())
                        if key == 'x':
                                motion_start_pub.publish()
                        if key == 'v':
                                comm.data = 1
                                ctrl_mode_pub1.publish(comm)
                                ctrl_mode_pub2.publish(comm)
                        if key == 'p':
                                comm.data = 0
                                ctrl_mode_pub1.publish(comm)
                                ctrl_mode_pub2.publish(comm)
                        if key == '\x03':
                                break
                        rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


#todo: set original hovering location as rosparam once /quadrotor/flight_state becomes 5