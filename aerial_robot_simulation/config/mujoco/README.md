# Shared MuJoCo scenes

Build the robot packages first, then start the example with one MuJoCo 3 server. 
Regenerate models left over from MuJoCo 2 before the first build.

For a single gimbalrotor (choose `quad` or `ynltilt`):

```bash
roslaunch gimbalrotor bringup.launch simulation:=true real_machine:=false \
  mujoco:=true airframe:=quad
```

The generated gimbalrotor models select a world with fluid density and viscosity
zero. The server is paced at real time by default; generated servo initial positions
come from the airframe's Servo.yaml. Hardware bringup continues to default to
MuJoCo disabled. `control_config:=GimbalrotorLQIControl` selects the quad LQI
simulation configuration.

Start the shared example:

```bash
roslaunch aerial_robot_simulation multi_mujoco.launch
```

`scene:=/absolute/path/scene.yaml` selects another manifest. The example contains
one gimbalrotor and one mini quadrotor. Each robot entry accepts `namespace`,
`package`, `airframe` (gimbalrotor only), `pos`, `yaw` in radians, `prefix`,
`model` (optional MJCF override), and `use_spinal` (default true).
`pos` translates the generated model's original root position.

Gimbalrotors also accept `control_config` per robot. A top-level `control_config`
or the launch argument selects the default for all gimbalrotors, for example
`control_config:=GimbalrotorLQIControl`. A per-robot selection takes precedence.
The matching controller YAML must exist for the selected airframe.

Top-level `world`, `attachments` and `robot_collisions` configure composition.
Relative paths resolve against the manifest directory; ROS `$(find package)`
substitutions are supported. The world owns the physics timestep. The old bridge's
`clock_pub_freq` and `physics_substeps` fields are rejected. `real_time_factor`
defaults to 1.0 and forwards to the upstream server's `realtime` argument;
values in (0, 1] and -1 (unrestricted speed) are supported.

To inspect the generated files without starting ROS nodes:

```bash
rosrun aerial_robot_simulation multi_mujoco.py --scene /absolute/path/scene.yaml \
  --output-dir /tmp/my_mujoco_scene --generate-only
```

The output contains `scene.xml`, `scene.launch` and `initial_joint_states.yaml`.
The last file collects gimbal Servo.yaml initial positions as **prefixed** MuJoCo
joint names with string values, matching the upstream ROS1 parser. ROS and URDF
joint names remain local. The generated launch creates one control plugin per
robot in `/mujoco_world/MujocoPlugins`, one thrust visualizer, and one server.
Each robot's bringup starts its controllers with `launch_mujoco:=false`.

Prefix lookup requires the dependency branch described above. Start a fresh
simulation for every test.

## Connected robots and payloads

Attachments are MJCF fragments spliced into the scene in the order listed, after
the robots. Their names are not prefixed, so they refer to robot bodies and sites
by their final prefixed names, such as `chain1/base_link` or `carrier1/fc`.

- `chain.yaml` welds the root bodies of two gimbalrotors (`chain.xml`).
- `payload.yaml` hangs an 80 g free payload from two mini quadrotors with
  spatial tendons (`payload.xml`).
- `free_pair.yaml` is the noodle pair without the noodle. It uses the same
  namespaces and spawn poses, so it serves as the control experiment.
- `studio.yaml` places one gimbalrotor in the scanned studio (`studio.xml`).

```bash
roslaunch aerial_robot_simulation multi_mujoco.launch headless:=false \
  scene:="$(rospack find aerial_robot_simulation)/config/mujoco/chain.yaml"
```

Each flight controller still controls only its own drone. For connected robots,
arm both, take off together and send compatible targets.

## Elastic connector (pool noodle)

`elastic_chain.yaml` joins `/elastic1` and `/elastic2` (quad gimbalrotors, 1.2 m
apart) with a 1.2 m, 94 g foam rod built from a `mujoco.elasticity.cable`
composite and clamped to both `base_link`s with welds (`elastic_chain.xml`):

```bash
roslaunch aerial_robot_simulation elastic_mujoco.launch            # viewer
roslaunch aerial_robot_simulation elastic_mujoco.launch headless:=true \
  control_config:=GimbalrotorLQIControl
```

`elastic_mujoco.launch` is `multi_mujoco.launch` with this scene selected and
the viewer on. **Engine plugins.** `mujoco_ros` does not load MuJoCo's plugin
directory, and a plugin has to be registered before the model is compiled. So
`multi_mujoco.py` reads the composed scene's `<extension>` block and preloads
the matching first-party libraries from `<devel>/lib/mujoco/mujoco_plugin`
(`mujoco.elasticity.*`, `mujoco.pid`, `mujoco.sensor.*` and `mujoco.sdf.*`). It
preloads them through `LD_PRELOAD`, which is set on the server node only. Any
other plugin name is rejected.

**What changed on MuJoCo 3.** The rod itself is unchanged. The end welds were
retuned, because MuJoCo 3.0 changed how a weld's stiffness follows the inertia
of the bodies it connects. The old `torquescale="0.1" solref="0.08 1"` became a
pin. The welds now use `solref="0.007 1"` with the default `torquescale`. This
reproduces the bending and twisting stiffness of the clamp flown on MuJoCo
2.3.7 (about 0.27 Nm/rad in pitch). The axial stiffness rises from 22 N/m, an
artifact of 2.3.7, to 1.5 kN/m, which is close to the real foam's. The comments
in `elastic_chain.xml` give the measurements and explain how to tune them.

Arm, take off and land both drones together. With the PID controller each
drone trims about 6 degrees in pitch against the clamp moment in hover.

## Physics checks without ROS

`mujoco_scene_physics_test` (built with `CATKIN_ENABLE_TESTING`) steps a
generated scene with external lift forces and no controllers:

```bash
rosrun aerial_robot_simulation multi_mujoco.py --generate-only --output-dir /tmp/noodle \
  --scene "$(rospack find aerial_robot_simulation)/config/mujoco/elastic_chain.yaml"
rosrun aerial_robot_simulation mujoco_scene_physics_test /tmp/noodle/scene.xml elastic
# For chain.yaml use "chain"; for payload.yaml use "payload".
```

Without a third argument the check loads `lib/mujoco/mujoco_plugin` from the
first `CMAKE_PREFIX_PATH` workspace that has it; a library path may be given
instead. The elastic check also requires both rod ends to stay on their drones.

## Repeatable flight check

The flight helper starts a fresh launch on a private local ROS master, verifies
`/use_sim_time`, arms and takes off each named robot, checks hover and ground-truth
publication, then lands. Every deadline uses wall time. Logs and the final JSON
result are written even when a flight fails; shutdown stops the launched process
group. Use an existing launch file that selects simulation and contains the
needed test overrides, including mocap `unhealth_level: 0` for the documented
startup race.

```bash
python3 aerial_robot_simulation/test/mujoco_flight_smoke.py \
  /tmp/my_mujoco_scene/scene.launch gimbal1 mini1 \
  --output /tmp/my_mujoco_flight --hold 5 --isolation
```

`--launch /path/to/scene.launch` also accepts the launch path explicitly.
`--port` selects an unused master port when required; the default is automatic.
`--isolation` checks that the second robot stays landed while the first takes off.
`--together` arms, takes off and lands all robots at once; use it for connected
scenes, for example `elastic1 elastic2 --together` on the elastic-connector scene.
The helper exits with status 0 only after every requested robot lands successfully.
