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
The helper exits with status 0 only after every requested robot lands successfully.
