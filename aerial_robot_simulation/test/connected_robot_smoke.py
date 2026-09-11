#!/usr/bin/env python3
"""Coordinated vertical flight of the supplied chain or payload pair.

Usage after starting the corresponding scene:
  python3 connected_robot_smoke.py carrier1 carrier2
  python3 connected_robot_smoke.py chain1 chain2
"""
import sys
import time

import rospy
from aerial_robot_msgs.msg import FlightNav
from multi_robot_smoke import Drone, wait


def run(names):
    rospy.init_node("connected_robot_smoke", anonymous=True)
    assert rospy.get_param("/use_sim_time", False), "Requires simulated time"
    drones = [Drone(name) for name in names]
    wait(lambda: all(d.pos is not None and d.state == 0 for d in drones), 30, "both landed")
    time.sleep(2)
    initial = [d.pos for d in drones]
    for d in drones:
        d.command("start")
    wait(lambda: all(d.state == 2 for d in drones), 15, "both armed")
    for d in drones:
        d.command("takeoff")
    wait(lambda: all(d.state == 5 for d in drones), 100, "both hovering")
    print("Hover:", [d.summary() for d in drones], flush=True)
    for d in drones:
        d.nav.publish(FlightNav(control_frame=FlightNav.WORLD_FRAME, target=FlightNav.BASELINK,
                               pos_z_nav_mode=FlightNav.POS_MODE, target_pos_z=1.0))
    wait(lambda: all(abs(d.pos[2]-1.0) < 0.15 for d in drones), 60, "both at 1 m")
    time.sleep(5)
    assert all(d.state == 5 and abs(d.pos[2]-1.0) < 0.15 for d in drones), [d.summary() for d in drones]
    assert all(abs(d.pos[0]-p[0]) < 0.2 and abs(d.pos[1]-p[1]) < 0.2 for d, p in zip(drones, initial)), [d.summary() for d in drones]
    print("Connected flight:", [d.summary() for d in drones], flush=True)
    for d in drones:
        d.command("land")
    wait(lambda: all(d.state == 0 for d in drones), 100, "both landed")
    print("PASS:", [d.summary() for d in drones], flush=True)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("Usage: connected_robot_smoke.py ROBOT_A ROBOT_B")
    run(sys.argv[1:])
