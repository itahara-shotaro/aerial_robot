#!/usr/bin/env python

from __future__ import print_function # for print function in python2
import socket
import sys, select, termios, tty

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3Stamped
from aerial_robot_msgs.msg import FlightNav
import rosgraph




msg = """
Instruction:

---------------------------

r:  arming motor (please do before takeoff)
t:  takeoff
l:  land
f:  force landing
h:  halt (force stop motor)

     q           w           e           [
(turn left)  (forward)  (turn right)  (move up)

     a           s           d           ]
(move left)  (backward) (move right) (move down)

baselink rotation (robots with final_target_baselink_rpy, e.g. gimbalrotor, dragon):

                 i
           (tilt forward)
     j                       k
(tilt left)             (tilt right)
                 m
            (tilt back)

o:  reset baselink rotation to level


Please don't have caps lock on.
CTRL+c to quit
---------------------------
"""

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def printMsg(msg, msg_len = 50):
        print(msg.ljust(msg_len) + "\r", end="")

def publishToAll(publishers):
        for publisher in publishers:
                publisher.publish(Empty())

def publishNav(nav_pub, nav_msg, success_msg):
        if nav_pub is None:
                return "navigation command ignored: set ~robot_ns to select a robot"

        nav_pub.publish(nav_msg)
        return success_msg

def publishRot(rot_pub, roll, pitch):
        if rot_pub is None:
                return "rotation command ignored: set ~robot_ns to select a robot"

        rot_msg = Vector3Stamped()
        rot_msg.header.stamp = rospy.Time.now()
        rot_msg.vector.x = roll
        rot_msg.vector.y = pitch
        rot_pub.publish(rot_msg)
        return "send baselink rot command: roll %.2f, pitch %.2f" % (roll, pitch)

def clamp(value, limit):
        return max(-limit, min(limit, value))

if __name__=="__main__":
        settings = termios.tcgetattr(sys.stdin)
        rospy.init_node("keyboard_command")
        robot_ns = rospy.get_param("~robot_ns", "");
        print(msg)

        master = rosgraph.Master('/rostopic')
        try:
                _, subs, _ = master.getSystemState()

        except socket.error:
                raise rospy.ROSException("Unable to communicate with master!")

        teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
        robot_namespaces = sorted(set(topic.split('/teleop')[0] for topic in teleop_topics))

        if not robot_namespaces:
                rospy.logwarn("No robots found; lifecycle commands will have no recipients")

        land_pubs = [rospy.Publisher(ns + '/teleop_command/land', Empty, queue_size=1) for ns in robot_namespaces]
        halt_pubs = [rospy.Publisher(ns + '/teleop_command/halt', Empty, queue_size=1) for ns in robot_namespaces]
        start_pubs = [rospy.Publisher(ns + '/teleop_command/start', Empty, queue_size=1) for ns in robot_namespaces]
        takeoff_pubs = [rospy.Publisher(ns + '/teleop_command/takeoff', Empty, queue_size=1) for ns in robot_namespaces]
        force_landing_pubs = [rospy.Publisher(ns + '/teleop_command/force_landing', Empty, queue_size=1) for ns in robot_namespaces]

        nav_robot_ns = robot_ns
        if not nav_robot_ns and len(robot_namespaces) == 1:
                nav_robot_ns = robot_namespaces[0]

        nav_pub = None
        rot_pub = None
        if nav_robot_ns:
                nav_pub = rospy.Publisher(nav_robot_ns + '/uav/nav', FlightNav, queue_size=1)
                rot_pub = rospy.Publisher(nav_robot_ns + '/final_target_baselink_rpy', Vector3Stamped, queue_size=1)
        elif len(robot_namespaces) > 1:
                rospy.logwarn("Multiple robots found; set ~robot_ns to enable navigation commands")
        else:
                rospy.logwarn("No robot selected; navigation commands are disabled")

        xy_vel   = rospy.get_param("xy_vel", 0.2)
        z_vel    = rospy.get_param("z_vel", 0.2)
        yaw_vel  = rospy.get_param("yaw_vel", 0.2)
        rot_step = rospy.get_param("rot_step", 0.02) # [rad] per key press
        max_tilt = rospy.get_param("max_tilt", 0.5) # [rad] limit for each of roll and pitch

        # the navigator only keeps the final target, so track it here and send absolute angles
        target_roll = 0.0
        target_pitch = 0.0

        motion_start_pub = rospy.Publisher('task_start', Empty, queue_size=1)

        try:
                while(True):
                        nav_msg = FlightNav()
                        nav_msg.control_frame = FlightNav.WORLD_FRAME
                        nav_msg.target = FlightNav.COG

                        key = getKey()

                        msg = ""

                        if key == 'l':
                                publishToAll(land_pubs)
                                target_roll = target_pitch = 0.0 # the navigator resets the rotation on landing
                                msg = "send land command"
                        if key == 'r':
                                publishToAll(start_pubs)
                                msg = "send motor-arming command"
                        if key == 'h':
                                publishToAll(halt_pubs)
                                target_roll = target_pitch = 0.0
                                msg = "send motor-disarming (halt) command"
                        if key == 'f':
                                publishToAll(force_landing_pubs)
                                target_roll = target_pitch = 0.0
                                msg = "send force landing command"
                        if key == 't':
                                publishToAll(takeoff_pubs)
                                msg = "send takeoff command"
                        if key == 'x':
                                motion_start_pub.publish()
                                msg = "send task-start command"
                        if key == 'w':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_x = xy_vel
                                msg = publishNav(nav_pub, nav_msg, "send +x vel command")
                        if key == 's':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_x = -xy_vel
                                msg = publishNav(nav_pub, nav_msg, "send -x vel command")
                        if key == 'a':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_y = xy_vel
                                msg = publishNav(nav_pub, nav_msg, "send +y vel command")
                        if key == 'd':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_y = -xy_vel
                                msg = publishNav(nav_pub, nav_msg, "send -y vel command")
                        if key == 'q':
                                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_omega_z = yaw_vel
                                msg = publishNav(nav_pub, nav_msg, "send +yaw vel command")
                        if key == 'e':
                                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_omega_z = -yaw_vel
                                msg = publishNav(nav_pub, nav_msg, "send -yaw vel command")
                        if key == '[':
                                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_z = z_vel
                                msg = publishNav(nav_pub, nav_msg, "send +z vel command")
                        if key == ']':
                                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_z = -z_vel
                                msg = publishNav(nav_pub, nav_msg, "send -z vel command")
                        if key == 'i':
                                target_pitch = clamp(target_pitch + rot_step, max_tilt)
                                msg = publishRot(rot_pub, target_roll, target_pitch)
                        if key == 'm':
                                target_pitch = clamp(target_pitch - rot_step, max_tilt)
                                msg = publishRot(rot_pub, target_roll, target_pitch)
                        if key == 'j':
                                target_roll = clamp(target_roll - rot_step, max_tilt)
                                msg = publishRot(rot_pub, target_roll, target_pitch)
                        if key == 'k':
                                target_roll = clamp(target_roll + rot_step, max_tilt)
                                msg = publishRot(rot_pub, target_roll, target_pitch)
                        if key == 'o':
                                target_roll = target_pitch = 0.0
                                msg = publishRot(rot_pub, target_roll, target_pitch)
                        if key == '\x03':
                                break

                        printMsg(msg)
                        rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
