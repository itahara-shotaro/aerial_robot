#!/usr/bin/env python3
"""Prototype: merge several single-robot MJCF models (as produced by
mujoco_model_generator.py) into one multi-robot MJCF.

MuJoCo 2.3.7 has neither <attach> nor <replicate>, and <include> is a plain
textual include, so every name has to be rewritten by hand to avoid collisions.

usage:
  merge_mujoco_models.py out.xml world.xml robot.xml:prefix:x,y,z[:yaw] ...
"""
import os
import sys
import math
import xml.etree.ElementTree as ET
from xml.dom import minidom

# (element tag, attribute) pairs that hold a name defined by the model
NAME_ATTRS = ["name", "joint", "site", "body", "mesh", "geom", "tendon", "jointinparent"]


def prefix_tree(root, prefix, meshdir):
    """rename everything the model defines/references, and absolutize mesh paths"""
    for elem in root.iter():
        if elem.tag == "mesh" and "file" in elem.attrib:
            elem.set("file", os.path.abspath(os.path.join(meshdir, elem.attrib["file"])))
        for attr in NAME_ATTRS:
            if attr in elem.attrib:
                elem.set(attr, prefix + elem.attrib[attr])


def yaw_quat(yaw):
    return "%f 0 0 %f" % (math.cos(yaw / 2.0), math.sin(yaw / 2.0))


def main():
    out_path, world_path = sys.argv[1], sys.argv[2]
    specs = sys.argv[3:]

    out_root = ET.Element("mujoco")
    out_root.set("model", "multi_robot")

    # the world file carries compiler/option/asset/worldbody defaults
    world = ET.parse(world_path).getroot()
    for elem in world:
        out_root.append(elem)

    compiler = out_root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(out_root, "compiler")
    # the per-robot models are generated with these; the world file is not
    compiler.set("autolimits", "true")
    compiler.set("balanceinertia", "true")

    out_asset = out_root.find("asset")
    if out_asset is None:
        out_asset = ET.SubElement(out_root, "asset")
    out_world = out_root.find("worldbody")
    if out_world is None:
        out_world = ET.SubElement(out_root, "worldbody")
    out_actuator = ET.SubElement(out_root, "actuator")
    out_sensor = ET.SubElement(out_root, "sensor")

    for spec in specs:
        parts = spec.split(":")
        path, prefix, pos = parts[0], parts[1], parts[2]
        yaw = float(parts[3]) if len(parts) > 3 else 0.0

        tree = ET.parse(path)
        root = tree.getroot()
        meshdir = os.path.dirname(os.path.abspath(path))
        prefix_tree(root, prefix, meshdir)

        for mesh in root.findall("./asset/mesh"):
            out_asset.append(mesh)

        for body in root.findall("./worldbody/body"):
            body.set("pos", pos.replace(",", " "))
            if yaw:
                body.set("quat", yaw_quat(yaw))
            out_world.append(body)

        for act in root.findall("./actuator/*"):
            out_actuator.append(act)
        for sen in root.findall("./sensor/*"):
            out_sensor.append(sen)

    xmlstr = minidom.parseString(ET.tostring(out_root)).toprettyxml(indent="  ")
    with open(out_path, "w") as f:
        f.write("\n".join([l for l in xmlstr.split("\n") if l.strip()]))
    print("wrote %s" % out_path)


if __name__ == "__main__":
    main()
