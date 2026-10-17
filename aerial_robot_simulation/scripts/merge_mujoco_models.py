#!/usr/bin/env python3
"""Compose generated aerial-robot MJCFs for a shared MuJoCo world.

Legacy CLI: merge_mujoco_models.py out.xml world.xml robot.xml:prefix:x,y,z[:yaw] ...
Positions are world translations added to each model's original root pose;
yaw (radians) rotates the entire model before translation.
"""
import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

NAME_ATTRS = {
    "name", "joint", "joint1", "joint2", "jointinparent", "site", "site1", "site2",
    "body", "body1", "body2", "mesh", "geom", "geom1", "geom2", "tendon",
    "tendon1", "tendon2", "material", "texture", "hfield", "target", "class",
    "childclass", "actuator", "objname", "refname", "cranksite", "slidersite",
    "refsite", "camera",
}
# Generated robots need a known schema for safe namespacing. Attachments are
# already authored in scene coordinates/names and are passed through as MJCF.
ROBOT_SECTIONS = {"asset", "worldbody", "actuator", "sensor", "tendon", "equality", "contact", "custom"}


def numbers(value, count, label):
    try:
        result = [float(x) for x in value]
    except (TypeError, ValueError) as exc:
        raise ValueError("%s must contain %d finite numbers" % (label, count)) from exc
    if len(result) != count or not all(math.isfinite(x) for x in result):
        raise ValueError("%s must contain %d finite numbers" % (label, count))
    return result


def resolve_assets(root, directory):
    compiler = root.find("compiler")
    settings = compiler.attrib if compiler is not None else {}
    for elem in root.iter():
        if "file" not in elem.attrib or elem.tag == "include":
            continue
        asset_dir = settings.get("meshdir" if elem.tag == "mesh" else "texturedir", settings.get("assetdir", ""))
        elem.set("file", str((Path(directory) / asset_dir / elem.get("file")).resolve()))


def prefix_tree(root, prefix, meshdir):
    """Rename MJCF definitions and references, preserving local ROS names."""
    resolve_assets(root, meshdir)
    for elem in root.iter():
        for attr in NAME_ATTRS.intersection(elem.attrib):
            elem.set(attr, prefix + elem.get(attr))


def load_fragment(path, stack=()):
    """Expand world/attachment includes relative to the including file."""
    path = Path(path).resolve()
    if path in stack:
        raise ValueError("Recursive MJCF include: %s" % path)
    root = ET.parse(str(path)).getroot()
    resolve_assets(root, path.parent)
    for parent in list(root.iter()):
        for child in list(parent):
            if child.tag == "include":
                index = list(parent).index(child)
                included = load_fragment(path.parent / child.get("file"), stack + (path,))
                parent.remove(child)
                for offset, elem in enumerate(included):
                    parent.insert(index + offset, elem)
    return root


def section(root, tag):
    found = root.find(tag)
    return found if found is not None else ET.SubElement(root, tag)


def merge_models(world_path, robots, attachments=(), robot_collisions=True):
    """robots: dictionaries with model, prefix, optional pos and yaw.

    Attachments are ordered MJCF fragments, not isolated robot models: complete
    sections are appended without namespacing or a section allowlist. Global
    settings/defaults in a fragment therefore apply to the whole final model.
    """
    if not robots:
        raise ValueError("At least one robot is required")
    prefixes = [robot["prefix"] for robot in robots]
    for i, prefix in enumerate(prefixes):
        if not prefix or any(prefix.startswith(other) or other.startswith(prefix) for other in prefixes[:i]):
            raise ValueError("Robot prefixes must be nonempty, unique, and non-overlapping")
    if robot_collisions and len(robots) > 29:
        raise ValueError("Collision filtering supports at most 29 robots")
    output = load_fragment(world_path)
    output.set("model", "multi_aerial_robot")
    compiler = section(output, "compiler")
    if compiler.get("angle", "degree") != "radian":
        raise ValueError("World must declare compiler angle=radian")
    compiler.set("angle", "radian")
    compiler.set("autolimits", "true")
    compiler.set("balanceinertia", "true")
    # All assets have absolute paths; do not inherit world path settings.
    for attr in ("meshdir", "texturedir", "assetdir"):
        compiler.attrib.pop(attr, None)

    for index, spec in enumerate(robots):
        path = Path(spec["model"]).resolve()
        root = ET.parse(str(path)).getroot()
        model_compiler = root.find("compiler")
        if model_compiler is None or model_compiler.get("angle", "degree") != "radian":
            raise ValueError("Robot models must declare compiler angle=radian: %s" % path)
        # The model generator appends a world include. The scene supplies that
        # world once. Other includes need to be expanded in the input model.
        for include in root.findall("include"):
            if Path(include.get("file", "")).name != "world.xml":
                raise ValueError("Unsupported robot include (expand it first): %s" % include.get("file"))
            root.remove(include)
        if any(elem.tag == "include" for elem in root.iter()):
            raise ValueError("Expand nested robot includes before composition")
        unsupported = {elem.tag for elem in root} - ROBOT_SECTIONS - {"compiler", "statistic", "default"}
        if unsupported:
            raise ValueError("Scene owns global settings; unsupported robot sections: %s" % sorted(unsupported))
        prefix_tree(root, spec["prefix"], path.parent)
        # Scope implicit defaults to this model, too.
        for defaults in root.findall("default"):
            if "class" not in defaults.attrib or defaults.get("class") == spec["prefix"] + "main":
                default_class = spec["prefix"] + "defaults"
                defaults.set("class", default_class)
                for body in root.findall("./worldbody/body"):
                    body.attrib.setdefault("childclass", default_class)
                for tag, default_tags in {
                    "actuator": None,
                    "equality": None,
                    "tendon": None,
                    "asset": {"mesh", "material"},
                    "contact": {"pair"},
                    "worldbody": {"geom", "site", "camera", "light"},
                }.items():
                    for elem in root.findall("./%s/*" % tag):
                        if default_tags is None or elem.tag in default_tags:
                            elem.attrib.setdefault("class", default_class)
            section(output, "default").append(defaults)
        pos = numbers(spec.get("pos", (0, 0, 0)), 3, "pos")
        try:
            yaw = float(spec.get("yaw", 0))
        except (TypeError, ValueError) as exc:
            raise ValueError("yaw must be finite") from exc
        if not math.isfinite(yaw):
            raise ValueError("yaw must be finite")
        c, s = math.cos(yaw), math.sin(yaw)
        for body in root.findall("./worldbody/body"):
            xyz = numbers(body.get("pos", "0 0 0").split(), 3, "body pos")
            body.set("pos", " ".join(map(str, (pos[0] + c*xyz[0] - s*xyz[1], pos[1] + s*xyz[0] + c*xyz[1], pos[2] + xyz[2]))))
            if any(attr in body.attrib for attr in ("euler", "axisangle", "xyaxes", "zaxis")):
                raise ValueError("Root body orientation must use quat")
            w, x, y, z = numbers(body.get("quat", "1 0 0 0").split(), 4, "body quat")
            cw, sz = math.cos(yaw / 2), math.sin(yaw / 2)
            body.set("quat", " ".join(map(str, (cw*w-sz*z, cw*x-sz*y, cw*y+sz*x, cw*z+sz*w))))
        if robot_collisions:
            bit = 1 << (index + 1)
            mask = ((1 << (len(robots) + 1)) - 1) ^ bit
            for geom in root.findall("./worldbody//geom"):
                if geom.get("contype", "1") == "0" and geom.get("conaffinity", "1") == "0":
                    continue
                geom.set("contype", str(bit))
                geom.set("conaffinity", str(mask))
        for elem in root:
            if elem.tag in ROBOT_SECTIONS:
                section(output, elem.tag).extend(list(elem))

    for path in attachments:
        # Splice complete top-level elements, like MJCF <include>. In particular,
        # do not flatten <extension>/<default> or discard section attributes
        # (e.g. <option viscosity="...">). This also preserves future MJCF
        # sections without teaching this merger their schema. MuJoCo remains
        # responsible for schema validation and repeated-section semantics.
        # Paths/includes are resolved above; names and plugin config stay local
        # to the authored scene, with no robot prefix applied.
        output.extend(list(load_fragment(path)))
    return output


def write_xml(root, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    # ElementTree.indent is only available after Python 3.8 (ROS Noetic).
    from xml.dom import minidom
    xml = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
    path.write_text("\n".join(line for line in xml.splitlines() if line.strip()) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output")
    parser.add_argument("world")
    parser.add_argument("robots", nargs="+")
    parser.add_argument("--attachments", action="append", default=[])
    args = parser.parse_args()
    robots = []
    for value in args.robots:
        parts = value.split(":")
        if len(parts) not in (3, 4):
            parser.error("robot must be model.xml:prefix:x,y,z[:yaw]")
        robots.append(dict(model=parts[0], prefix=parts[1], pos=parts[2].split(","), yaw=parts[3] if len(parts) == 4 else 0))
    try:
        write_xml(merge_models(args.world, robots, args.attachments), args.output)
    except (ValueError, OSError, ET.ParseError) as exc:
        parser.error(str(exc))
    print("wrote %s" % args.output)


if __name__ == "__main__":
    main()
