#!/usr/bin/env python3
"""Launch a fresh local simulation, take off, hold, and land each named robot.

Accepts either ``SCENE.launch ROBOT_NS ...`` or ``--launch SCENE.launch ROBOT_NS ...``.
A private ROS master port is selected automatically; --port requests a specific
unused port. Flight commands are enabled only after this master's /use_sim_time
parameter is true. Deadlines use the wall clock so a stalled /clock fails the run.

The launch must select simulation and supply any test-only parameter overrides.
Artifacts are launch.log and result.json beneath --output. A failing check or
Ctrl-C still records the result and stops the entire launched process group.
"""

import argparse
import json
import http.client
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import time
import traceback
import xmlrpc.client


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("targets", nargs="+", metavar="LAUNCH_OR_ROBOT_NS",
                        help="launch file followed by namespaces, or namespaces with --launch")
    parser.add_argument("--launch", dest="launch_file", help="existing launch file")
    parser.add_argument("--output", required=True, type=Path, help="directory for logs and result.json")
    parser.add_argument("--port", type=int, default=0, help="unused local ROS master port (default: automatic)")
    parser.add_argument("--hold", type=float, default=5.0, help="hover duration in wall seconds")
    parser.add_argument("--isolation", action="store_true",
                        help="check the second robot stays landed while the first takes off")
    parser.add_argument("--together", action="store_true",
                        help="arm, take off and land all robots at once (robots joined by a connector)")
    args = parser.parse_args(argv)
    if args.launch_file:
        args.names = args.targets
    else:
        if len(args.targets) < 2:
            parser.error("provide a launch file and at least one robot namespace")
        args.launch_file, *args.names = args.targets
    args.launch_file = Path(args.launch_file).expanduser().resolve()
    if not args.launch_file.is_file():
        parser.error("launch file does not exist: %s" % args.launch_file)
    args.names = [name.strip("/") for name in args.names]
    if (len(set(args.names)) != len(args.names)
            or any(not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*", name)
                   for name in args.names)):
        parser.error("robot namespaces must be unique, nonempty ROS names")
    if not 0 <= args.port <= 65535:
        parser.error("--port must be in 0..65535")
    if args.together and args.isolation:
        parser.error("--isolation requires sequential takeoff; omit --together")
    if not math.isfinite(args.hold) or args.hold <= 0:
        parser.error("--hold must be a positive finite duration")
    args.output = args.output.expanduser().resolve()
    return args


def unused_port(requested):
    """Refuse an occupied requested port; otherwise select an ephemeral port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.bind(("127.0.0.1", requested))
        return listener.getsockname()[1]


def remember_process_groups(process):
    """Track roslaunch children, which ROS starts in their own sessions."""
    now = time.monotonic()
    if now - getattr(process, "last_group_scan", 0) < 0.25:
        return
    process.last_group_scan = now
    process.owned_groups = getattr(process, "owned_groups", {process.pid})
    processes = {}
    for path in Path("/proc").iterdir():
        if not path.name.isdecimal():
            continue
        try:
            # The process name in parentheses may itself contain spaces.
            fields = (path / "stat").read_text().rsplit(")", 1)[1].split()
            processes[int(path.name)] = (int(fields[1]), int(fields[2]))
        except (OSError, ValueError, IndexError):
            continue
    descendants = {process.pid}
    while True:
        children = {pid for pid, (parent, _) in processes.items() if parent in descendants}
        if children <= descendants:
            break
        descendants.update(children)
    process.owned_groups.update(processes[pid][1] for pid in descendants if pid in processes)


def group_alive(group):
    try:
        os.killpg(group, 0)
        return True
    except ProcessLookupError:
        return False


def wait_for_master(process, master_uri):
    """Bound master startup with wall time before rospy registers the node."""
    class LocalTransport(xmlrpc.client.Transport):
        def make_connection(self, host):
            return http.client.HTTPConnection(host, timeout=1)

    deadline = time.monotonic() + 35
    with xmlrpc.client.ServerProxy(master_uri, transport=LocalTransport()) as master:
        while time.monotonic() < deadline:
            remember_process_groups(process)
            if process.poll() is not None:
                raise RuntimeError("roslaunch exited before master startup")
            try:
                code, _, _ = master.getPid("/mujoco3_flight_check")
                if code == 1:
                    return
            except (OSError, xmlrpc.client.Error):
                pass
            time.sleep(0.05)
    raise RuntimeError("Timeout waiting for private ROS master")


def stop_process_group(process):
    """Stop children even if roslaunch itself has already exited."""
    process.last_group_scan = 0
    remember_process_groups(process)
    groups = process.owned_groups
    for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        # Give roslaunch the first chance to shut down its detached ROS nodes.
        targets = {process.pid} if sig == signal.SIGINT else groups
        for group in targets:
            try:
                os.killpg(group, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            process.poll()
            if not any(group_alive(group) for group in groups):
                break
            time.sleep(0.1)
        else:
            continue
        break
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2)


class FlightSmoke:
    def __init__(self, args, process, rospy):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Empty, UInt8
        from tf.transformations import euler_from_quaternion

        self.args = args
        self.process = process
        self.rospy = rospy
        self.empty_message = Empty
        self.euler_from_quaternion = euler_from_quaternion
        self.data = {name: dict(state=None, odom=None, truth=0, max_tilt=0.0,
                                max_gimbal_speed=0.0, last=0.0) for name in args.names}
        self.subscribers = []
        self.publishers = {}
        self.landing = set()
        for name in args.names:
            self.subscribers.extend([
                rospy.Subscriber("/%s/flight_state" % name, UInt8, self.on_state, name),
                rospy.Subscriber("/%s/uav/baselink/odom" % name, Odometry, self.on_odom, name),
                rospy.Subscriber("/%s/ground_truth" % name, Odometry, self.on_truth, name),
                rospy.Subscriber("/%s/joint_states" % name, JointState, self.on_joints, name),
            ])
            self.publishers[name] = {
                command: rospy.Publisher("/%s/teleop_command/%s" % (name, command),
                                         Empty, queue_size=1)
                for command in ("start", "takeoff", "land")}

    def on_state(self, message, name):
        self.data[name]["state"] = message.data

    def on_odom(self, message, name):
        state = self.data[name]
        orientation = message.pose.pose.orientation
        roll, pitch, _ = self.euler_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w))
        state["odom"] = message
        state["last"] = time.monotonic()
        state["max_tilt"] = max(state["max_tilt"], abs(roll), abs(pitch))

    def on_truth(self, message, name):
        self.data[name]["truth"] += 1

    def on_joints(self, message, name):
        speeds = [abs(speed) for joint, speed in zip(message.name, message.velocity)
                  if joint.startswith("gimbal")]
        if speeds:
            state = self.data[name]
            state["max_gimbal_speed"] = max(state["max_gimbal_speed"], max(speeds))

    def snapshot(self):
        result = {}
        for name, state in self.data.items():
            odom = state["odom"]
            position = odom.pose.pose.position if odom is not None else None
            result[name] = dict(
                state=state["state"],
                position=[position.x, position.y, position.z] if position is not None else None,
                truth_messages=state["truth"], max_tilt=state["max_tilt"],
                max_gimbal_speed=state["max_gimbal_speed"])
        return result

    def check_health(self):
        remember_process_groups(self.process)
        if self.process.poll() is not None:
            raise RuntimeError("roslaunch exited with status %s" % self.process.returncode)
        if self.rospy.is_shutdown():
            raise RuntimeError("ROS shutdown during flight")
        text = (self.args.output / "launch.log").read_text(errors="replace")
        if "invalid matrix product" in text:
            raise RuntimeError("KF Eigen assertion in launch.log")
        for name, state in self.data.items():
            odom = state["odom"]
            if odom is not None:
                position, orientation = odom.pose.pose.position, odom.pose.pose.orientation
                linear, angular = odom.twist.twist.linear, odom.twist.twist.angular
                if not all(math.isfinite(value) for value in
                           (position.x, position.y, position.z, orientation.x,
                            orientation.y, orientation.z, orientation.w,
                            linear.x, linear.y, linear.z, angular.x, angular.y, angular.z)):
                    raise RuntimeError("Non-finite odometry for " + name)
            unexpected_stop = state["state"] == 6 and name not in self.landing
            if state["state"] == 17 or unexpected_stop or state["max_tilt"] > 1.0:
                raise RuntimeError("Flight failure for %s: %s" % (name, self.snapshot()))

    def wait(self, predicate, seconds, label):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.check_health()
            if predicate():
                return
            time.sleep(0.05)
        raise RuntimeError("Timeout waiting for %s: %s" % (label, self.snapshot()))

    def command(self, name, command):
        publisher = self.publishers[name][command]
        self.wait(lambda: publisher.get_num_connections() > 0, 10,
                  "%s %s subscriber" % (name, command))
        if command == "land":
            self.landing.add(name)
        publisher.publish(self.empty_message())

    def clock_publishers(self):
        import rosgraph
        publishers = rosgraph.Master(self.rospy.get_name()).getSystemState()[0]
        owners = dict(publishers).get("/clock", [])
        if len(owners) != 1:
            raise RuntimeError("Expected one /clock publisher, found %s" % owners)
        return owners

    def take_off_together(self):
        for name in self.args.names:
            self.command(name, "start")
        self.wait(lambda: all(state["state"] == 2 for state in self.data.values()), 15, "armed all")
        for name in self.args.names:
            self.command(name, "takeoff")
        self.wait(lambda: all(state["state"] == 5 for state in self.data.values()), 60, "hover all")
        print("hover", json.dumps(self.snapshot()), flush=True)

    def run(self):
        self.wait(lambda: all(state["state"] == 0 and state["odom"] is not None
                              and time.monotonic() - state["last"] < 1
                              for state in self.data.values()), 35, "landed odometry")
        # Allow normal IMU bias calibration and controller model initialization.
        calibration_end = time.monotonic() + 8
        while time.monotonic() < calibration_end:
            self.check_health()
            time.sleep(0.05)
        self.wait(lambda: all(state["state"] == 0 for state in self.data.values()),
                  2, "calibration completion")
        initial = self.snapshot()
        print("initial", json.dumps(initial), flush=True)
        owners = self.clock_publishers()
        if self.args.together:
            self.take_off_together()
        for index, name in enumerate([] if self.args.together else self.args.names):
            self.command(name, "start")
            self.wait(lambda: self.data[name]["state"] == 2, 15, "armed " + name)
            self.command(name, "takeoff")
            self.wait(lambda: self.data[name]["state"] == 5, 60, "hover " + name)
            if self.args.isolation and index == 0 and len(self.args.names) > 1:
                other = self.args.names[1]
                current = self.snapshot()[other]
                if current["state"] != 0 or math.dist(current["position"], initial[other]["position"]) >= 0.08:
                    raise RuntimeError("Inactive robot moved: %s" % self.snapshot())
            print("hover", name, json.dumps(self.snapshot()), flush=True)
        hold_end = time.monotonic() + self.args.hold
        while time.monotonic() < hold_end:
            self.wait(lambda: all(state["state"] == 5 and time.monotonic() - state["last"] < 1
                                  for state in self.data.values()), 1, "hover hold")
            time.sleep(0.1)
        hover = self.snapshot()
        for name in self.args.names:
            state = self.data[name]
            if state["odom"].pose.pose.position.z <= initial[name]["position"][2] + 0.25:
                raise RuntimeError("Insufficient takeoff height: %s" % self.snapshot())
            if state["truth"] <= 20:
                raise RuntimeError("Missing ground-truth stream: %s" % self.snapshot())
            if self.args.together:
                continue
            self.command(name, "land")
            self.wait(lambda: self.data[name]["state"] == 0, 60, "land " + name)
        if self.args.together:
            for name in self.args.names:
                self.command(name, "land")
            self.wait(lambda: all(state["state"] == 0 for state in self.data.values()), 60, "land all")
        self.clock_publishers()
        return dict(initial=initial, hover=hover, final=self.snapshot(), clock_publishers=owners)


def run(args):
    result = dict(status="FAIL", names=args.names)
    started = time.monotonic()
    process = None
    rospy = None
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "launch.log").open("w") as launch_log:
        try:
            port = unused_port(args.port)
            master_uri = "http://127.0.0.1:%d" % port
            result["master_uri"] = master_uri
            os.environ.update(ROS_MASTER_URI=master_uri, ROS_HOSTNAME="127.0.0.1",
                              ROS_IP="127.0.0.1", ROS_HOME=str(args.output / "ros"))
            process = subprocess.Popen(["roslaunch", str(args.launch_file)], stdout=launch_log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            wait_for_master(process, master_uri)
            # Import after selecting the private master, before creating any ROS objects.
            import rospy as ros_client
            rospy = ros_client
            rospy.init_node("mujoco3_flight_check", anonymous=True, disable_signals=True)
            deadline = time.monotonic() + 35
            while not rospy.has_param("/use_sim_time"):
                if process.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError("Simulation did not set /use_sim_time before startup deadline")
                time.sleep(0.05)
            if rospy.get_param("/use_sim_time") is not True:
                raise RuntimeError("Refusing flight commands: /use_sim_time must be true")
            result.update(FlightSmoke(args, process, rospy).run())
            result["status"] = "PASS"
        except KeyboardInterrupt:
            result["error"] = "Interrupted"
        except Exception as error:
            result["error"] = str(error)
            traceback.print_exc()
        finally:
            # Keep a second interrupt from skipping process cleanup or result.json.
            handlers = {sig: signal.signal(sig, signal.SIG_IGN)
                        for sig in (signal.SIGINT, signal.SIGTERM)}
            try:
                if rospy is not None:
                    rospy.signal_shutdown("Flight smoke check complete")
                if process is not None:
                    stop_process_group(process)
            except Exception as error:
                result.update(status="FAIL", cleanup_error=str(error))
            finally:
                for sig, handler in handlers.items():
                    signal.signal(sig, handler)
            result["seconds"] = round(time.monotonic() - started, 3)
            (args.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result), flush=True)
    return 0 if result["status"] == "PASS" else 1


def main(argv=None):
    args = parse_args(argv)

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    previous_handler = signal.signal(signal.SIGTERM, interrupted)
    try:
        return run(args)
    finally:
        signal.signal(signal.SIGTERM, previous_handler)


if __name__ == "__main__":
    raise SystemExit(main())
