#!/usr/bin/env python3
"""Exercise two robots in an already running, *unconnected* MuJoCo scene.

Usage: python3 multi_robot_smoke.py gimbal1 mini1
Checks separate controllers, takeoff isolation, simultaneous hover, a waypoint
sent to only the first robot, and independent landing. Uses wall-time deadlines.
"""
import math
import sys
import time

import rospy
from aerial_robot_msgs.msg import FlightNav
from controller_manager_msgs.srv import ListControllers
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, UInt8


class Drone:
    def __init__(self, namespace):
        self.ns = "/" + namespace.strip("/")
        self.state = None
        self.pos = None
        self.updated = 0
        self.state_sub = rospy.Subscriber(self.ns + "/flight_state", UInt8, self.on_state)
        self.odom_sub = rospy.Subscriber(self.ns + "/uav/baselink/odom", Odometry, self.on_odom)
        self.commands = {name: rospy.Publisher(self.ns + "/teleop_command/" + name, Empty, queue_size=1)
                         for name in ("start", "takeoff", "land")}
        self.nav = rospy.Publisher(self.ns + "/uav/nav", FlightNav, queue_size=1)

    def on_state(self, msg):
        self.state = msg.data

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y, p.z)
        self.updated = time.monotonic()

    def summary(self):
        return dict(namespace=self.ns, state=self.state, position=self.pos)

    def command(self, command):
        wait(lambda: self.commands[command].get_num_connections() > 0, 10, "command subscriber " + self.ns)
        self.commands[command].publish(Empty())


def wait(predicate, timeout, label):
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError("Timeout: " + label)


def distance(a, b):
    return math.sqrt(sum((x-y)**2 for x, y in zip(a, b)))


def run(names):
    rospy.init_node("multi_robot_smoke", anonymous=True)
    assert rospy.get_param("/use_sim_time", False), "This check requires simulated time"
    drones = [Drone(name) for name in names]
    a, b = drones
    wait(lambda: all(d.pos is not None and d.state == 0 for d in drones), 30, "both landed with odometry")
    for d in drones:
        service = d.ns + "/controller_manager/list_controllers"
        rospy.wait_for_service(service, timeout=10)
        controllers = rospy.ServiceProxy(service, ListControllers)().controller
        assert any(c.name == "attitude_controller" and c.state == "running" for c in controllers), d.summary()
    time.sleep(2)
    initial = [d.pos for d in drones]
    print("Initial:", [d.summary() for d in drones], flush=True)
    a.command("start")
    wait(lambda: a.state == 2, 15, "first armed")
    a.command("takeoff")
    wait(lambda: a.state == 5, 90, "first hovering")
    assert a.pos[2] > initial[0][2] + 0.25, a.summary()
    assert b.state == 0 and distance(b.pos, initial[1]) < 0.08, b.summary()
    print("Takeoff isolation:", [d.summary() for d in drones], flush=True)
    b.command("start")
    wait(lambda: b.state == 2, 15, "second armed")
    b.command("takeoff")
    wait(lambda: b.state == 5, 90, "second hovering")
    time.sleep(3)
    hold = b.pos
    origin = a.pos
    target = (origin[0], origin[1] + 0.4, origin[2])
    nav = FlightNav(control_frame=FlightNav.WORLD_FRAME, target=FlightNav.BASELINK,
                    pos_xy_nav_mode=FlightNav.POS_MODE, target_pos_x=target[0], target_pos_y=target[1])
    a.nav.publish(nav)
    wait(lambda: distance(a.pos[:2], target[:2]) < 0.12, 45, "first waypoint")
    time.sleep(3)
    assert a.state == b.state == 5, [d.summary() for d in drones]
    assert distance(b.pos, hold) < 0.15, b.summary()
    assert all(time.monotonic() - d.updated < 1 for d in drones), "Stale odometry"
    print("Waypoint isolation:", [d.summary() for d in drones], flush=True)
    a.command("land")
    wait(lambda: a.state == 0, 90, "first landed")
    assert b.state == 5, b.summary()
    b.command("land")
    wait(lambda: b.state == 0, 90, "second landed")
    print("PASS:", [d.summary() for d in drones], flush=True)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("Usage: multi_robot_smoke.py ROBOT_A ROBOT_B")
    run(sys.argv[1:])
