#!/usr/bin/env python3
"""Select gimbalrotor's world and write MuJoCo 3 initial servo positions."""
from pathlib import Path
import xml.etree.ElementTree as ET
import argparse
import math
import yaml


def configure(package_dir, world):
    package_dir = Path(package_dir)
    for airframe in ('quad', 'ynltilt'):
        directory = package_dir / 'mujoco' / airframe
        model = directory / 'robot.xml'
        root = ET.parse(str(model)).getroot()
        includes = root.findall('include')
        world_includes = [item for item in includes if Path(item.get('file', '')).name == 'world.xml']
        if len(world_includes) != 1:
            raise ValueError('Expected one generated world include in %s' % model)
        world_includes[0].set('file', str(Path(world).resolve()))
        ET.ElementTree(root).write(str(model), encoding='utf-8', xml_declaration=True)
        servos = yaml.safe_load((package_dir / 'config' / airframe / 'Servo.yaml').read_text())['servo_controller']
        positions = {}
        for group in servos.values():
            if not isinstance(group, dict):
                continue
            for key, controller in group.items():
                if not key.startswith('controller') or not isinstance(controller, dict):
                    continue
                simulation = dict(group.get('simulation', {}))
                simulation.update(controller.get('simulation', {}))
                initial = simulation.get('init_value')
                if isinstance(initial, bool) or not isinstance(initial, (int, float)) or not math.isfinite(initial):
                    raise ValueError('Servo %s requires a finite simulation init_value' % controller['name'])
                # mujoco_ros ROS1 consumes a map<string,string>, including scalars.
                positions[controller['name']] = str(float(initial))
        (directory / 'initial_joint_states.yaml').write_text(yaml.safe_dump(
            {'initial_joint_positions': {'joint_map': positions}}, sort_keys=True))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('package_dir')
    parser.add_argument('world')
    args = parser.parse_args()
    configure(args.package_dir, args.world)
