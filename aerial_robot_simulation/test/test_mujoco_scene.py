#!/usr/bin/env python3
"""Composition regressions independent of ROS master and MuJoCo installation."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock
import yaml
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from merge_mujoco_models import merge_models

MODEL = '''<mujoco>
<compiler angle="radian" meshdir="meshes"/>
<default><geom friction="0.5 0.01 0.001"/></default>
<asset><mesh name="shape" file="body.stl"/><texture name="skin" file="skin.png"/>
<material name="paint" texture="skin"/></asset>
<worldbody><body name="base" pos="1 0 0.2"><freejoint/>
<geom name="shape" mesh="shape" material="paint"/><site name="fc"/>
<body name="tip"><joint name="joint"/><site name="tip"/></body></body></worldbody>
<actuator><motor name="rotor1" site="fc"/></actuator>
<sensor><framepos name="pose" objtype="site" objname="fc" reftype="body" refname="base"/></sensor>
<tendon><spatial name="wire"><site site="fc"/><site site="tip"/></spatial></tendon>
<equality><joint name="lock" joint1="joint"/></equality>
<contact><exclude body1="base" body2="tip"/></contact>
<include file="world.xml"/>
</mujoco>'''


class SceneTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.world = self.root / "world.xml"
        self.world.write_text('<mujoco><compiler angle="radian"/><worldbody><geom name="floor"/></worldbody></mujoco>')
        self.model = self.root / "robot.xml"
        self.model.write_text(MODEL)

    def robot(self, prefix, **kwargs):
        return dict(model=str(self.model), prefix=prefix, **kwargs)

    def test_isolates_references_defaults_assets_and_collision_masks(self):
        merged = merge_models(self.world, [self.robot("a/"), self.robot("b/")])
        self.assertEqual([x.get("body1") for x in merged.findall("./contact/exclude")], ["a/base", "b/base"])
        self.assertEqual([x.get("joint1") for x in merged.findall("./equality/joint")], ["a/joint", "b/joint"])
        self.assertEqual([x.get("site") for x in merged.findall("./tendon/spatial/site")], ["a/fc", "a/tip", "b/fc", "b/tip"])
        self.assertEqual(merged.find("./sensor/framepos").get("objname"), "a/fc")
        self.assertEqual(merged.find("./sensor/framepos").get("refname"), "a/base")
        self.assertEqual(merged.find("./asset/mesh").get("file"), str(self.root / "meshes/body.stl"))
        self.assertEqual(merged.find("./asset/material").get("texture"), "a/skin")
        for path in ("./asset/material", "./asset/mesh", "./tendon/spatial", "./equality/joint"):
            self.assertEqual(merged.find(path).get("class"), "a/defaults")
        self.assertEqual(merged.find("./worldbody/body").get("childclass"), "a/defaults")
        geoms = merged.findall("./worldbody/body/geom")
        self.assertEqual([(x.get("contype"), x.get("conaffinity")) for x in geoms], [("2", "5"), ("4", "3")])

    def test_composes_pose_and_preserves_free_roots(self):
        import math
        merged = merge_models(self.world, [self.robot("a/", pos=[2, 3, 4], yaw=math.pi/2)])
        body = merged.find("./worldbody/body")
        for actual, expected in zip(map(float, body.get("pos").split()), [2, 4, 4.2]):
            self.assertAlmostEqual(actual, expected)
        self.assertIsNotNone(body.find("freejoint"))
        self.assertAlmostEqual(float(body.get("quat").split()[3]), math.sqrt(0.5))

    def test_preserves_payload_and_cross_robot_constraints(self):
        fragment = self.root / "payload.xml"
        fragment.write_text('<mujoco><worldbody><body name="payload"><freejoint/></body></worldbody><equality><weld body1="a/base" body2="b/base"/></equality></mujoco>')
        merged = merge_models(self.world, [self.robot("a/"), self.robot("b/")], [fragment])
        self.assertEqual(merged.find("./equality/weld").get("body2"), "b/base")
        self.assertEqual(len(merged.findall("./worldbody/body/freejoint")), 3)

    def test_preserves_plugin_declarations_instances_and_composites(self):
        fragment = self.root / "elastic.xml"
        fragment.write_text('''<mujoco>
<extension><plugin plugin="mujoco.elasticity.cable">
  <instance name="foam"><config key="bend" value="1e5"/></instance>
</plugin></extension>
<worldbody><composite prefix="noodle/" type="cable" count="25 1 1" size="1.2">
  <plugin instance="foam"/><geom type="capsule" size="0.03"/>
</composite></worldbody>
<equality><connect body1="noodle/B_last" body2="a/base" anchor="0.05 0 0"/></equality>
</mujoco>''')
        merged = merge_models(self.world, [self.robot("a/")], [fragment])
        for authored, actual in zip(ET.parse(str(fragment)).getroot(), list(merged)[-3:]):
            self.assertEqual(ET.tostring(authored), ET.tostring(actual))
        self.assertEqual(merged.find("./extension/plugin").get("plugin"), "mujoco.elasticity.cable")

    def test_passes_through_future_sections_attributes_and_attachment_order(self):
        first, second = self.root / "first.xml", self.root / "second.xml"
        first.write_text('<mujoco><option iterations="70"/><default class="foam"><geom density="30"/></default>'
                         '<future_feature mode="custom"><setting value="unchanged"/></future_feature></mujoco>')
        second.write_text('<mujoco><option iterations="90"/><visual><global elevation="-20"/></visual></mujoco>')
        merged = merge_models(self.world, [self.robot("a/")], [first, second])
        self.assertEqual([elem.tag for elem in list(merged)[-5:]],
                         ["option", "default", "future_feature", "option", "visual"])
        self.assertEqual([elem.get("iterations") for elem in merged.findall("option")], ["70", "90"])
        self.assertEqual(merged.find("./default[@class='foam']/geom").get("density"), "30")
        self.assertEqual(merged.find("future_feature").get("mode"), "custom")
        # The merger preserves unknown syntax; the engine decides if it is valid.
        self.assertEqual(merged.find("./future_feature/setting").get("value"), "unchanged")

    def test_attachment_includes_keep_plugins_and_resolve_assets(self):
        directory = self.root / "fragments"
        directory.mkdir()
        (directory / "elastic.xml").write_text('<mujocoinclude><extension><plugin plugin="mujoco.elasticity.cable"/></extension>'
                                               '<asset><mesh name="connector" file="connector.stl"/></asset></mujocoinclude>')
        fragment = self.root / "attachment.xml"
        fragment.write_text('<mujoco><include file="fragments/elastic.xml"/></mujoco>')
        merged = merge_models(self.world, [self.robot("a/")], [fragment])
        self.assertEqual(merged.find("./extension/plugin").get("plugin"), "mujoco.elasticity.cable")
        self.assertEqual(merged.find("./asset/mesh[@name='connector']").get("file"), str(directory / "connector.stl"))
        self.assertFalse(merged.findall(".//include"))
        (directory / "elastic.xml").write_text('<mujoco><include file="../attachment.xml"/></mujoco>')
        with self.assertRaisesRegex(ValueError, "Recursive MJCF include"):
            merge_models(self.world, [self.robot("a/")], [fragment])

    def test_rejects_overlapping_ownership_and_invalid_inputs(self):
        for prefixes in (("a/", "a/"), ("a", "a/"), ("", "b/")):
            with self.assertRaises(ValueError):
                merge_models(self.world, [self.robot(x) for x in prefixes])
        with self.assertRaises(ValueError):
            merge_models(self.world, [self.robot("a/", pos=[0, float("nan"), 0])])
        self.model.write_text(MODEL.replace('<include file="world.xml"/>', '<keyframe><key/></keyframe>'))
        with self.assertRaises(ValueError):
            merge_models(self.world, [self.robot("a/")])


class LaunchTests(unittest.TestCase):
    setUp = SceneTests.setUp
    def generate(self, robots, **extras):
        from multi_mujoco import generate
        scene = self.root / "scene.yaml"
        scene.write_text(yaml.safe_dump(dict(world=str(self.world), robots=robots, **extras)))
        with mock.patch("multi_mujoco.rospkg.RosPack") as rospack:
            rospack.return_value.get_path.return_value = str(self.root)
            return ET.parse(str(generate(scene, self.root / "output"))).getroot()

    def entry(self, namespace):
        return dict(namespace=namespace, package="mini_quadrotor", model=str(self.model))

    def test_launch_has_one_physics_owner_and_independent_bringups(self):
        launch = self.generate([self.entry("a"), self.entry("b")])
        nodes = launch.findall("node")
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].get("ns"), "mujoco_world")
        self.assertEqual(nodes[0].get("required"), "true")
        includes = launch.findall("include")
        self.assertEqual(len(includes), 2)
        for include, ns in zip(includes, ["a", "b"]):
            args = {arg.get("name"): arg.get("value") for arg in include}
            self.assertEqual(args["robot_ns"], ns)
            self.assertEqual(args["launch_mujoco"], "false")
            self.assertEqual(args["demo"], "false")
            self.assertEqual(args["real_machine"], "false")
        params = {param.get("name"): param.get("value") for param in launch.findall("param")}
        self.assertEqual(params["/a/simulation/model_prefix"], "a/")
        self.assertEqual(params["/b/simulation/model_prefix"], "b/")
        self.assertEqual(yaml.safe_load(launch.find("rosparam").text), ["/a", "/b"])

    def test_rejects_duplicate_namespaces_and_bad_scene_fields(self):
        with self.assertRaises(ValueError):
            self.generate([self.entry("a"), self.entry("a")])
        with self.assertRaises(ValueError):
            self.generate([self.entry("mujoco_world")])
        with self.assertRaises(ValueError):
            self.generate([self.entry("a")], robot_collisions="false")
        with self.assertRaises(ValueError):
            self.generate([self.entry("a")], attachments="payload.xml")


if __name__ == "__main__":
    unittest.main()
