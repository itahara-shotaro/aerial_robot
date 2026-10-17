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

# MuJoCo scenes

Shared MuJoCo 3 scenes (several robots in one server, connectors, payloads and
the elastic pool-noodle example) are documented in
[config/mujoco/README.md](config/mujoco/README.md).
