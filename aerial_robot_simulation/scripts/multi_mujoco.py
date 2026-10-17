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


def initial_positions(robot, package_path):
    """Translate Servo.yaml simulation defaults into the upstream string map."""
    positions = {}
    if robot["package"] == "gimbalrotor":
        servo = package_path / "config" / robot["airframe"] / "Servo.yaml"
        if not servo.is_file():
            raise ValueError("Servo configuration missing: %s" % servo)
        config = yaml.safe_load(servo.read_text())
        for group in config.get("servo_controller", {}).values():
            if not isinstance(group, dict):
                continue
            for controller in group.values():
                if not isinstance(controller, dict) or "name" not in controller:
                    continue
                simulation = dict(group.get("simulation", {}))
                simulation.update(controller.get("simulation", {}))
                if "init_value" not in simulation:
                    raise ValueError("Servo %s has no simulation init_value" % controller["name"])
                initial = simulation["init_value"]
                if isinstance(initial, bool) or not isinstance(initial, (int, float)) or not math.isfinite(initial):
                    raise ValueError("Servo initial positions must be finite numbers")
                positions[robot["prefix"] + controller["name"]] = str(float(initial))
    return positions


# First-party engine plugins shipped with MuJoCo, by the second component of the
# plugin name (mujoco.<group>.<name>), and the library under lib/mujoco/mujoco_plugin.
ENGINE_PLUGIN_LIBRARIES = {"elasticity": "elasticity", "pid": "actuator", "sensor": "sensor", "sdf": "sdf_plugin"}


def engine_plugin_libraries(model, search_path=None):
    """Return the plugin libraries that must be registered before loading model.

    mujoco_ros never calls mj_loadAllPluginLibraries, and a MujocoPlugin loads
    after the model, so the server has to preload these libraries.
    """
    names = sorted({plugin.get("plugin", "") for plugin in model.findall("./extension/plugin")})
    if not names:
        return []
    if search_path is None:
        search_path = os.environ.get("CMAKE_PREFIX_PATH", "").split(os.pathsep)
    libraries = []
    for name in names:
        parts = name.split(".")
        if len(parts) < 2 or parts[0] != "mujoco" or parts[1] not in ENGINE_PLUGIN_LIBRARIES:
            raise ValueError("Unknown MuJoCo engine plugin %s: only first-party mujoco.* plugins are preloaded" % name)
        filename = "lib%s.so" % ENGINE_PLUGIN_LIBRARIES[parts[1]]
        candidates = [Path(prefix) / "lib/mujoco/mujoco_plugin" / filename for prefix in search_path if prefix]
        found = next((path for path in candidates if path.is_file()), None)
        if found is None:
            raise ValueError("MuJoCo plugin library %s for %s not found under CMAKE_PREFIX_PATH" % (filename, name))
        if str(found) not in libraries:
            libraries.append(str(found))
    return libraries


def generate(scene_path, output_dir, headless=True, real_time_factor=1.0, control_config=None):
    if not math.isfinite(real_time_factor) or not (real_time_factor == -1 or 0 < real_time_factor <= 1):
        raise ValueError("real_time_factor must be in (0, 1], or -1 for unrestricted speed")
    scene_path = Path(scene_path).resolve()
    output_dir = Path(output_dir).resolve()
    scene = yaml.safe_load(scene_path.read_text())
    if not isinstance(scene, dict) or not isinstance(scene.get("robots"), list) or not scene["robots"]:
        raise ValueError("Scene must contain a nonempty robots list")
    unknown = set(scene) - {"world", "robots", "attachments", "robot_collisions",
                            "control_config"}
    if unknown:
        raise ValueError("Unknown scene fields: %s" % sorted(unknown))
    if not isinstance(scene.get("robot_collisions", True), bool):
        raise ValueError("robot_collisions must be a YAML boolean")
    if not isinstance(scene.get("attachments", []), list):
        raise ValueError("attachments must be a list of MJCF paths")
    package_paths = rospkg.RosPack()
    robots = []
    namespaces = set()
    joint_positions = {}
    control_config = control_config or scene.get("control_config")
    if control_config is not None and (not isinstance(control_config, str) or not control_config):
        raise ValueError("control_config must be a nonempty string")
    for entry in scene["robots"]:
        if (not isinstance(entry, dict) or not isinstance(entry.get("namespace"), str)
                or not isinstance(entry.get("package"), str)):
            raise ValueError("Each robot must specify a namespace and package")
        if "prefix" in entry and not isinstance(entry["prefix"], str):
            raise ValueError("prefix must be a string")
        unknown = set(entry) - {"namespace", "package", "airframe", "model", "pos", "yaw", "prefix", "control_config", "use_spinal"}
        if unknown:
            raise ValueError("Unknown robot fields: %s" % sorted(unknown))
        robot = dict(entry)
        if "control_config" in robot and (not isinstance(robot["control_config"], str) or not robot["control_config"]):
            raise ValueError("control_config must be a nonempty string")
        ns = robot["namespace"].strip("/")
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", ns) or ns in namespaces or ns == "mujoco_world":
            raise ValueError("Robot namespace must be a unique simple ROS name, excluding mujoco_world: %s" % ns)
        namespaces.add(ns)
        robot["namespace"] = ns
        if "use_spinal" in robot and not isinstance(robot["use_spinal"], bool):
            raise ValueError("use_spinal must be a YAML boolean")
        robot.setdefault("use_spinal", True)
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
        joint_positions.update(initial_positions(robot, package_path))
        robots.append(robot)
    world = resolve_path(scene.get("world", "$(find aerial_robot_simulation)/config/mujoco/world.xml"), scene_path.parent)
    attachments = [resolve_path(path, scene_path.parent) for path in scene.get("attachments", [])]
    model_path = output_dir / "scene.xml"
    model = merge_models(world, robots, attachments, scene.get("robot_collisions", True))
    preload = engine_plugin_libraries(model)
    write_xml(model, model_path)

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
        if robot["package"] == "gimbalrotor":
            selected_control = robot.get("control_config", control_config)
            if selected_control:
                if not isinstance(selected_control, str):
                    raise ValueError("control_config must be a string")
                args["control_config"] = selected_control
        elif "control_config" in robot:
            raise ValueError("mini_quadrotor does not accept control_config")
        args["use_spinal"] = str(robot["use_spinal"]).lower()
        for name, value in args.items():
            ET.SubElement(include, "arg", name=name, value=value)
        ET.SubElement(launch, "param", name="/%s/simulation/model_prefix" % ns, value=robot["prefix"], type="str")
    # All plugins share one model and one /clock, while controller managers and
    # ROS joint names remain per robot. Prefixes affect only MuJoCo name lookup.
    plugins = []
    for robot in robots:
        plugins.append(dict(
            type="mujoco_ros_control/MujocoRosControlPlugin",
            robot_namespace=robot["namespace"],
            hardware=dict(type="mujoco_ros_control/" + (
                "AerialRobotHWSim" if robot["use_spinal"] else "DefaultAerialRobotHWSim"),
                control_period=0.001)))
    plugins.append(dict(type="aerial_robot_simulation/ThrustVisualizerPlugin",
                        min_force=0.001, rgba=[0.0, 0.8, 1.0, 0.9]))
    config = ET.SubElement(launch, "rosparam", param="/mujoco_world/MujocoPlugins")
    config.text = yaml.safe_dump(plugins)
    # Upstream ROS1 expects map<string,string>, including scalar positions.
    # Prefixing these keys is essential: URDF/ROS joint names remain local.
    initial_path = output_dir / "initial_joint_states.yaml"
    initial_path.write_text(yaml.safe_dump(dict(initial_joint_positions=dict(joint_map=joint_positions))))
    server = ET.SubElement(launch, "include", file=package_paths.get_path("aerial_robot_simulation") + "/launch/mujoco_server.launch")
    server_args = dict(server_ns="mujoco_world", mujoco_model=str(model_path),
                       headless=str(headless).lower(), real_time_factor=str(real_time_factor),
                       initial_joint_states=str(initial_path), engine_plugins=" ".join(preload))
    for name, value in server_args.items():
        ET.SubElement(server, "arg", name=name, value=value)
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
    parser.add_argument("--control-config", help="Gimbalrotor controller config (per-robot scene values take precedence)")
    # roslaunch appends __name/__log remappings; reject misspelled CLI options.
    args = parser.parse_args([arg for arg in sys.argv[1:] if not (arg.startswith("__") and ":=" in arg)])
    output = args.output_dir or tempfile.mkdtemp(prefix="aerial_mujoco_")
    try:
        launch = generate(args.scene, output, args.headless == "true", args.real_time_factor, args.control_config)
    except (ValueError, KeyError, OSError, ET.ParseError, yaml.YAMLError, rospkg.ResourceNotFound) as exc:
        parser.error(str(exc))
    print("Generated %s" % launch, flush=True)
    if args.generate_only:
        return
    # Hand process/signal ownership to roslaunch, including Ctrl-C shutdown.
    os.execvp("roslaunch", ["roslaunch", str(launch)])


if __name__ == "__main__":
    main()
