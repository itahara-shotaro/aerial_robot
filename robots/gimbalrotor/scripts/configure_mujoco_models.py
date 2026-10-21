#!/usr/bin/env python3
"""Select gimbalrotor's world, restore the URDF collisions and write MuJoCo 3 initial servo positions."""
from pathlib import Path
import xml.etree.ElementTree as ET
import argparse
import math
import re
import subprocess
import yaml


def rpy_to_quat(roll, pitch, yaw):
    """URDF fixed-axis rpy (R = Rz Ry Rx) as a (w, x, y, z) quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def urdf_collisions(xacro_path):
    """Primitive collision geoms of every link in the URDF, as MJCF attribute dicts.

    The upstream generator discards the URDF <collision> tags and collides each link's visual mesh instead.
    A detailed mesh can have a flat underside made of many coplanar hull vertices, so MuJoCo's plane contact
    hops between them as the robot rocks on the ground. Gazebo uses the URDF collisions, which are simple.
    Returns {link: [attrs] | None}; None means the link has a collision that is not a primitive.
    """
    urdf = subprocess.run(['rosrun', 'xacro', 'xacro', str(xacro_path)], check=True,
                          stdout=subprocess.PIPE).stdout
    collisions = {}
    for link in ET.fromstring(urdf).findall('link'):
        shapes = []
        for index, collision in enumerate(link.findall('collision')):
            geometry = collision.find('geometry')[0]
            if geometry.tag == 'box':
                size = [float(value) / 2 for value in geometry.get('size').split()]
                attrs = {'type': 'box', 'size': ' '.join(map(str, size))}
            elif geometry.tag == 'cylinder':
                attrs = {'type': 'cylinder',
                         'size': '%s %s' % (geometry.get('radius'), float(geometry.get('length')) / 2)}
            elif geometry.tag == 'sphere':
                attrs = {'type': 'sphere', 'size': geometry.get('radius')}
            else:
                shapes = None
                break
            origin = collision.find('origin')
            xyz = [float(value) for value in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            rpy = [float(value) for value in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            attrs['pos'] = ' '.join(map(str, xyz))
            attrs['quat'] = ' '.join(map(str, rpy_to_quat(*rpy)))
            attrs['name'] = '%s_collision%d' % (link.get('name'), index)
            attrs['rgba'] = '0 0 0 0' # make the collision geom invisible
            shapes.append(attrs)
        collisions[link.get('name')] = shapes
    return collisions


def restore_collisions(root, collisions):
    """Collide the URDF shapes; keep the generated meshes as visual-only geoms. Safe to run repeatedly."""
    for body in root.iter('body'):
        link = body.get('name')
        if link not in collisions or collisions[link] is None:
            continue
        for geom in body.findall('geom'):
            if re.match(r'%s_collision\d+$' % re.escape(link), geom.get('name', '')):
                body.remove(geom)
        meshes = [geom for geom in body.findall('geom') if geom.get('type') == 'mesh']
        for mesh in meshes:
            mesh.set('contype', '0')
            mesh.set('conaffinity', '0')
        for attrs in collisions[link]:
            geom = ET.Element('geom', dict(attrs, contype='1', conaffinity='0'))
            if meshes:
                geom.tail = meshes[-1].tail  # keep the generated file's layout
                body.insert(list(body).index(meshes[-1]) + 1, geom)
            else:
                body.insert(0, geom)


def configure(package_dir, world):
    package_dir = Path(package_dir)
    converted = yaml.safe_load((package_dir / 'config' / 'mujoco_model.yaml').read_text())['gimbalrotor']
    xacros = dict(zip(converted['filename'], converted['input']))
    for airframe in ('quad', 'ynltilt'):
        directory = package_dir / 'mujoco' / airframe
        model = directory / 'robot.xml'
        root = ET.parse(str(model)).getroot()
        includes = root.findall('include')
        world_includes = [item for item in includes if Path(item.get('file', '')).name == 'world.xml']
        if len(world_includes) != 1:
            raise ValueError('Expected one generated world include in %s' % model)
        world_includes[0].set('file', str(Path(world).resolve()))
        restore_collisions(root, urdf_collisions(package_dir / xacros[airframe]))
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
