#!/usr/bin/env python3
"""Generate and launch a shared MuJoCo world from a YAML scene manifest."""
import argparse
import math
import os
from pathlib import Path
import re
import sys
import tempfile
import xml.etree.ElementTree as ET

import roslaunch
import rospkg
import yaml

# Also works through catkin's devel-space script relay.
sys.path.insert(0, str(Path(rospkg.RosPack().get_path("aerial_robot_simulation")) / "scripts"))
from merge_mujoco_models import merge_models, write_xml


def resolve_path(value, base):
    value = roslaunch.substitution_args.resolve_args(str(value))
    return str((base / os.path.expanduser(value)).resolve())


def generate(scene_path, output_dir, headless=True, real_time_factor=1.0):
    if not math.isfinite(real_time_factor):
        raise ValueError("real_time_factor must be finite")
    scene_path = Path(scene_path).resolve()
    output_dir = Path(output_dir).resolve()
    scene = yaml.safe_load(scene_path.read_text())
    if not isinstance(scene, dict) or not isinstance(scene.get("robots"), list) or not scene["robots"]:
        raise ValueError("Scene must contain a nonempty robots list")
    unknown = set(scene) - {"world", "robots", "attachments", "robot_collisions",
                            "clock_pub_freq", "physics_substeps"}
    if unknown:
        raise ValueError("Unknown scene fields: %s" % sorted(unknown))
    if not isinstance(scene.get("robot_collisions", True), bool):
        raise ValueError("robot_collisions must be a YAML boolean")
    if not isinstance(scene.get("attachments", []), list):
        raise ValueError("attachments must be a list of MJCF paths")
    package_paths = rospkg.RosPack()
    robots = []
    namespaces = set()
    for entry in scene["robots"]:
        if not isinstance(entry, dict) or not isinstance(entry.get("namespace"), str):
            raise ValueError("Each robot must specify a namespace and package")
        if "prefix" in entry and not isinstance(entry["prefix"], str):
            raise ValueError("prefix must be a string")
        unknown = set(entry) - {"namespace", "package", "airframe", "model", "pos", "yaw", "prefix"}
        if unknown:
            raise ValueError("Unknown robot fields: %s" % sorted(unknown))
        robot = dict(entry)
        ns = robot["namespace"].strip("/")
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", ns) or ns in namespaces or ns == "mujoco_world":
            raise ValueError("Robot namespace must be a unique simple ROS name, excluding mujoco_world: %s" % ns)
        namespaces.add(ns)
        robot["namespace"] = ns
        package = robot["package"]
        if package not in ("gimbalrotor", "mini_quadrotor"):
            raise ValueError("Supported robot packages: gimbalrotor, mini_quadrotor")
        package_path = Path(package_paths.get_path(package))
        if package == "gimbalrotor":
            robot.setdefault("airframe", "quad")
            if robot["airframe"] not in ("quad", "ynltilt"):
                raise ValueError("Generated gimbalrotor models support quad and ynltilt")
            model = package_path / "mujoco" / robot["airframe"] / "robot.xml"
        else:
            if "airframe" in robot:
                raise ValueError("mini_quadrotor does not accept airframe")
            model = package_path / "mujoco/robot.xml"
        robot["model"] = resolve_path(robot.get("model", str(model)), scene_path.parent)
        if not Path(robot["model"]).is_file():
            raise ValueError("Model missing: %s. Build %s with catkin build -j4 first." % (robot["model"], package))
        robot.setdefault("prefix", ns + "/")
        robots.append(robot)
    world = resolve_path(scene.get("world", "$(find aerial_robot_simulation)/config/mujoco/world.xml"), scene_path.parent)
    attachments = [resolve_path(path, scene_path.parent) for path in scene.get("attachments", [])]
    model_path = output_dir / "scene.xml"
    write_xml(merge_models(world, robots, attachments, scene.get("robot_collisions", True)), model_path)

    launch = ET.Element("launch")
    ET.SubElement(launch, "param", name="/use_sim_time", value="true")
    for robot in robots:
        ns = robot["namespace"]
        include = ET.SubElement(launch, "include", file=package_paths.get_path(robot["package"]) + "/launch/bringup.launch")
        args = dict(robot_ns=ns, simulation="true", real_machine="false", mujoco="true", launch_mujoco="false", headless="true")
        if robot["package"] == "gimbalrotor":
            args["airframe"] = robot["airframe"]
        else:
            args["demo"] = "false"
        for name, value in args.items():
            ET.SubElement(include, "arg", name=name, value=value)
        ET.SubElement(launch, "param", name="/%s/simulation/model_prefix" % ns, value=robot["prefix"], type="str")
    ET.SubElement(launch, "param", name="/mujoco_world/simulation/robot_hw_sim_plugin_name", value="mujoco_ros_control/MultiAerialRobotHWSim")
    robot_param = ET.SubElement(launch, "rosparam", param="/mujoco_world/simulation/robots")
    robot_param.text = yaml.safe_dump(["/" + robot["namespace"] for robot in robots])
    # The existing bridge supplies physics, viewer, pacing and the single clock.
    node = ET.SubElement(launch, "node", pkg="mujoco_ros_control", type="mujoco_ros_control", name="mujoco_ros_control", ns="mujoco_world", output="screen", required="true")
    node_params = dict(mujoco_model_path=str(model_path), headless=str(headless).lower(), real_time_factor=str(real_time_factor))
    # Optional: shorten the physics step without changing the control rate.
    for key in ("clock_pub_freq", "physics_substeps"):
        if key in scene:
            if not isinstance(scene[key], int) or isinstance(scene[key], bool) or scene[key] < 1:
                raise ValueError("%s must be a positive integer" % key)
            node_params[key] = str(scene[key])
    for name, value in node_params.items():
        ET.SubElement(node, "param", name=name, value=value)
    launch_path = output_dir / "scene.launch"
    write_xml(launch, launch_path)
    return launch_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--output-dir", help="Keep generated scene.xml and scene.launch here")
    parser.add_argument("--generate-only", action="store_true")
    parser.add_argument("--headless", choices=("true", "false"), default="true")
    parser.add_argument("--real-time-factor", type=float, default=1.0)
    # roslaunch appends __name/__log remappings; reject misspelled CLI options.
    args = parser.parse_args([arg for arg in sys.argv[1:] if not (arg.startswith("__") and ":=" in arg)])
    output = args.output_dir or tempfile.mkdtemp(prefix="aerial_mujoco_")
    try:
        launch = generate(args.scene, output, args.headless == "true", args.real_time_factor)
    except (ValueError, KeyError, OSError, ET.ParseError, yaml.YAMLError, rospkg.ResourceNotFound) as exc:
        parser.error(str(exc))
    print("Generated %s" % launch, flush=True)
    if args.generate_only:
        return
    # Hand process/signal ownership to roslaunch, including Ctrl-C shutdown.
    os.execvp("roslaunch", ["roslaunch", str(launch)])


if __name__ == "__main__":
    main()
