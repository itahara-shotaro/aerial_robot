# Studio scan in Gazebo

The Scaniverse capture is available as a textured static model
in [gazebo_model/world/studio.world](gazebo_model/world/studio.world). Select it
with the existing `worldtype` argument, for example:

```bash
source devel/setup.bash
roslaunch gimbalrotor bringup.launch simulation:=True real_machine:=False \
  mujoco:=False headless:=False \
  worldtype:="$(rospack find aerial_robot_simulation)/gazebo_model/world/studio.world"
```

Run this from the workspace root after building/sourcing the workspace. Other
robots exposing `worldtype` in their bringup launch file can use the same world.
The default world remains `empty.world`.

The mesh retains the scan's metre scale and origin, with coordinates converted
from Y-up to Z-up: `(x, y, z)` becomes `(x, -z, y)`. The floor is near world
`z=0`; use the bringup arguments `spawn_x`, `spawn_y`, `spawn_z`, and `spawn_yaw`
to choose a starting pose. The COLLADA mesh references the adjacent `studio.jpg`
texture; no original archive or external model download is needed at runtime.

The scan provides visual geometry and an invisible flat ground plane provides 
floor collision. Scanned walls, furniture, and ceiling do not provide collision
geometry.

## Studio scan in MuJoCo

Launch the same studio environment with one gimbalrotor using the scene launcher:

```bash
roslaunch aerial_robot_simulation multi_mujoco.launch headless:=false \
  scene:="$(rospack find aerial_robot_simulation)/config/mujoco/studio.yaml"
```

[studio.yaml](config/mujoco/studio.yaml) selects
[studio.xml](config/mujoco/studio.xml) and a `quad` robot at the scan origin,
under `/gimbalrotor1`. Arm, take off, and land using this namespace's usual
commands. Change the scene's robot list, positions, or airframes to choose another
robot setup. Other MuJoCo scenes can select this environment with
`world: $(find aerial_robot_simulation)/config/mujoco/studio.xml`.

The OBJ asset contains the same 249,999 triangles, coordinates, normals, and UVs
as the Gazebo mesh. Its PNG texture is derived from the original JPEG atlas at
4096 by 4096 pixels to limit texture memory. The scan is visual-only and the
invisible plane at `z=0` supplies floor collision, as in Gazebo. Assets resolve
relative to the world file and are installed with the package's configuration.

# Multiple aerial robots in MuJoCo

Run one MuJoCo world with a separate `aerial_robot_base_node`, spinal attitude
controller, controller manager, servo bridge (where needed), and ROS namespace
for each drone. Supported generated models are `gimbalrotor` (`quad`, `ynltilt`)
and `mini_quadrotor`. A scene may mix these types or repeat them.

## Build and run

From your sourced ROS1 workspace:

```bash
catkin build aerial_robot_simulation gimbalrotor mini_quadrotor -j4
source devel/setup.bash
roslaunch aerial_robot_simulation multi_mujoco.launch headless:=false
```

The default scene contains `/gimbal1` and `/mini1`, separated by two metres.
`headless:=false` opens the MuJoCo viewer; it does not open one RViz per drone.
Use `headless:=true` for unattended runs. `real_time_factor:=1.0` requests normal
simulation speed, subject to available CPU time.

The existing model generator caches its output directory. After modifying a
robot's URDF, regenerate its MJCF explicitly, for example:

```bash
rosrun mujoco_ros_control mujoco_model_generator.py \
  "$(rospack find mini_quadrotor)/config/mujoco_model.yaml"
```

Replace `mini_quadrotor` with `gimbalrotor` to regenerate both supported gimbal
models. The scene launcher uses these generated `mujoco/.../robot.xml` files.

## Command each robot

No drone takes off automatically. Address commands to its explicit namespace:

```bash
rostopic pub -1 /gimbal1/teleop_command/start std_msgs/Empty '{}'
# Wait for /gimbal1/flight_state to become 2 (Armed).
rostopic pub -1 /gimbal1/teleop_command/takeoff std_msgs/Empty '{}'
# Wait for state 5 (Hovering) before sending navigation commands.
rostopic pub -1 /gimbal1/uav/nav aerial_robot_msgs/FlightNav \
  '{control_frame: 0, target: 0, pos_xy_nav_mode: 2, target_pos_x: -1.0, target_pos_y: 0.4}'
rostopic pub -1 /gimbal1/teleop_command/land std_msgs/Empty '{}'
```

Use `/mini1/...` for the other drone. Observe each robot on
`/<namespace>/uav/baselink/odom`, `/<namespace>/ground_truth`,
`/<namespace>/joint_states`, and `/<namespace>/flight_state`.
Each `/<namespace>/controller_manager/list_controllers` service reports its own
running `attitude_controller`. The single physics process and `/clock` publisher
are at `/mujoco_world/mujoco_ros_control`.

## Describe a scene

Copy [config/mujoco/independent.yaml](config/mujoco/independent.yaml) and launch it:

```bash
roslaunch aerial_robot_simulation multi_mujoco.launch scene:=/absolute/path/my_scene.yaml
```

```yaml
robots:
  - namespace: drone1
    package: gimbalrotor
    airframe: quad
    pos: [-1.0, 0, 0]
    yaw: 0.0
  - namespace: drone2
    package: mini_quadrotor
    pos: [1.0, 0, 0]
# Optional:
# world: /path/to/world.xml
# attachments: [payload.xml]
# robot_collisions: true
```

- `namespace` is a unique simple ROS name, such as `drone1`; `mujoco_world` is
  reserved. Each robot has its own URDF and controller parameters.
- `pos` translates the generated model's original pose in world metres. Its
  original spawn height (usually 0.2 m) is retained. `yaw` is in radians and
  rotates the complete model before translation.
- `prefix` defaults to `<namespace>/`. Only MJCF names receive this prefix;
  ROS joint names remain local names such as `gimbal1`. Explicit prefixes must
  be nonempty and neither equal nor be a prefix of another robot's prefix.
- `model` can override the generated MJCF path. It must match the selected
  robot's URDF and controller configuration, use radians and quaternion root
  orientations, and retain `fc`, `acc`, `gyro`, `mag`, and contiguous
  `rotor1` ... `rotorN` actuator names before prefixing.
- Relative paths resolve beside the scene YAML; `$(find package)` substitutions
  also work. An attachment fragment's includes resolve beside that fragment.
- Drone–drone collisions are enabled by default with separate collision bits
  for each robot, retaining disabled self-collision. This supports up to 29
  drones. `robot_collisions: false` preserves the original model masks.
- Custom worlds must declare `<compiler angle="radian"/>`. The world owns
  gravity, integration options, lighting, and global settings.

To inspect or reuse the generated files without starting ROS nodes:

```bash
rosrun aerial_robot_simulation multi_mujoco.py \
  --scene /absolute/path/my_scene.yaml --output-dir /tmp/my_scene --generate-only
roslaunch /tmp/my_scene/scene.launch
```

The normal launcher prints the temporary directory containing `scene.xml` and
`scene.launch` and retains it for inspection. The legacy positional interface of
`merge_mujoco_models.py` is also available for composing MJCF only.

## Connect robots and payloads

```bash
roslaunch aerial_robot_simulation multi_mujoco.launch \
  scene:="$(rospack find aerial_robot_simulation)/config/mujoco/chain.yaml" headless:=false
# Or use config/mujoco/payload.yaml.
```

[chain.xml](config/mujoco/chain.xml) welds the root bodies of two gimbalrotors,
preserving their initial relative pose. Add more robots and weld pairs to
extend the chain. Both bodies keep their free joints.

[payload.xml](config/mujoco/payload.xml) adds a separate 80 g free payload and
connects it to two mini quadrotors using spatial tendons. The cables have a
slack interval, spring stiffness and damping, and an upper length limit. The
example uses each drone's `fc` site as its attachment point. For different
attachment locations, add named sites to the robot MJCF body before merging.

Attachment files are MJCF fragments, spliced into the scene in the order listed
in `attachments`. The merger preserves complete top-level sections and their
attributes instead of filtering sections or moving only their children. This
supports `extension`, `default`, `option`, `visual`, composite geometry, and
future MJCF sections supported by your engine. MuJoCo validates the resulting
schema and applies its usual rules for repeated sections. Nested includes are
expanded and asset paths are resolved relative to their source files.

Attachment names, plugin identifiers, instance references, and plugin config
values are not prefixed or otherwise interpreted. Give each attachment's named
objects, plugin instances, default classes, and composite prefixes unique names.
Reference the final prefixed robot names, e.g. `carrier1/fc` or
`chain1/base_link`. A `connect` equality provides a ball connection in place of
a weld. Global options and unscoped defaults in an attachment affect the whole
scene; retain the scene's radian angle convention. Paths embedded in opaque
plugin `<config value="...">` strings are passed through unchanged: use absolute
paths if a plugin interprets these strings as filenames.

Robot model imports still use stricter processing because their names and poses
must be transformed. The merger retains and prefixes their constraints,
materials, textures, and named defaults, and scopes implicit defaults to each
robot. It skips the generated models' `world.xml` includes because the scene
provides one world. Other robot includes must be expanded first; global robot
options and keyframes are rejected.

Connections couple the physics; each flight controller still controls its own
drone. For the supplied connected examples, arm both robots, take off together,
and send compatible position targets. At the default takeoff height the payload
cables may remain slack; command both drones to 1 m to lift it. Payload mass,
cable geometry, gains, and trajectories must suit the drones' available thrust.
There is no additional cooperative payload controller or automatic allocation
of a shared payload's mass.

## Thick elastic connectors (MuJoCo engine plugins)

[elastic_chain.yaml](config/mujoco/elastic_chain.yaml) connects two gimbalrotors
with a 1.2 m long, 6 cm diameter capsule chain using `mujoco.elasticity.cable`.
Unlike the payload example's spatial tendons, this connector has material
geometry and bending/torsional elasticity. Its radius, density, `bend` and
`twist` moduli are illustrative settings, not measured pool-noodle properties.
Tune these in [elastic_chain.xml](config/mujoco/elastic_chain.xml); `count`
controls the discretization. Each end uses a ball attachment. Use welds if you
want to constrain the endpoint orientation as well. The full example reserves
collision bit 30 for the connector, allowing contact with the floor and itself
while its endpoint constraints transfer loads into the drones. This avoids
contact forces from overlapping geometry at the mounts.

A minimal plugin declaration and connector look like:

```xml
<extension>
  <plugin plugin="mujoco.elasticity.cable"/>
</extension>
<worldbody>
  <composite prefix="noodle/" type="cable" curve="s" count="25 1 1"
             size="1.2" offset="-0.6 0 0.2" initial="free">
    <plugin plugin="mujoco.elasticity.cable">
      <config key="twist" value="5e4"/>
      <config key="bend" value="1e5"/>
    </plugin>
    <joint kind="main" damping="0.01"/>
    <geom type="capsule" size="0.03" density="30"/>
  </composite>
</worldbody>
```

MuJoCo plugins must be registered **before model loading**; XML declarations do
not load shared libraries. See [MuJoCo's plugin registration documentation](https://mujoco.readthedocs.io/en/2.3.7/programming/extension.html#registration).
The current ROS bridge does not call `mj_loadPluginLibrary`.
[elastic_mujoco.launch](launch/elastic_mujoco.launch) sets `LD_LIBRARY_PATH` and
`LD_PRELOAD` for the scene launcher and its child processes, preserving existing
entries. It selects the elastic-chain scene and opens the GUI by default:

```bash
roslaunch aerial_robot_simulation elastic_mujoco.launch
```

Use `headless:=true` for a headless run or `scene:=/path/to/scene.yaml` for another
scene. `mujoco_dir` defaults to the bridge's bundled `build/mujoco-2.3.7`
directory; `plugin_library` defaults to its `bin/mujoco_plugin/libelasticity.so`.
Both paths can be overridden, and `real_time_factor` is forwarded as usual.
The environment additions do not modify your shell. For a custom simulator
executable, call `mj_loadPluginLibrary` before `mj_loadXML`.
Use libraries built for the same MuJoCo version as your simulator. Other engine
plugins follow the same attachment mechanism; no additional merger allowlist
entry is necessary.

The native dynamics check can load the library directly:

```bash
MUJOCO_CONNECTOR_DIST="$(rospack find mujoco_ros_control)/build/mujoco-2.3.7"
rosrun aerial_robot_simulation multi_mujoco.py \
  --scene "$(rospack find aerial_robot_simulation)/config/mujoco/elastic_chain.yaml" \
  --output-dir /tmp/elastic_scene --generate-only
rosrun aerial_robot_simulation mujoco_scene_physics_test \
  /tmp/elastic_scene/scene.xml elastic \
  "$MUJOCO_CONNECTOR_DIST/bin/mujoco_plugin/libelasticity.so"
```

The example has passed model compilation and the native bending/load-transfer
check. Stable coordinated flight with the existing controller gains has not
been established. Inspect the model in the viewer and tune the connector and
controllers for your application. `connected_robot_smoke.py elastic1 elastic2`
is available to exercise coordinated vertical flight and landing.

## Validation

Composition and launch-generation tests, plus a C++ regression for interleaved
attitude estimators, run through catkin's test target:

```bash
catkin build aerial_robot_simulation --no-deps -j4 --make-args run_tests
```

The Python composition tests can also run directly:

```bash
python3 -m unittest discover -s "$(rospack find aerial_robot_simulation)/test" -v
```

For a running, unconnected default scene, the following check arms and flies
both drones, tests takeoff/waypoint/landing isolation, and lands them:

```bash
python3 "$(rospack find aerial_robot_simulation)/test/multi_robot_smoke.py" gimbal1 mini1
```

For the payload or chain scene, a coordinated vertical flight check is available:

```bash
python3 "$(rospack find aerial_robot_simulation)/test/connected_robot_smoke.py" carrier1 carrier2
# Use chain1 chain2 for the chain scene.
```

When `CATKIN_ENABLE_TESTING` is enabled, `mujoco_scene_physics_test` also checks
coupling without ROS controllers, using generated example scenes:

```bash
rosrun aerial_robot_simulation mujoco_scene_physics_test /tmp/my_scene/scene.xml payload
# Use chain for the generated chain example.
```

Single-robot bringup remains available with its existing defaults. To manually
include a robot in a custom shared-world launch, set `robot_ns`, `mujoco:=true`,
`simulation:=true`, `real_machine:=false`, and `launch_mujoco:=false`; this starts
its controller spawner without starting another physics process. Set
`/<namespace>/simulation/model_prefix` and list its absolute namespace in the
shared plugin's `simulation/robots` parameter, as in the generated launch file.

The existing single-mini-quadrotor regression is also supported:

```bash
PYTHONNOUSERSITE=1 rostest mini_quadrotor flight_control.test mujoco:=true headless:=true
```

`PYTHONNOUSERSITE=1` uses the system NumPy version; it avoids the installed
ROS Noetic `ros_numpy` package's use of the removed `numpy.float` alias when a
newer NumPy is installed in the user's Python directory.
