# Camera extrinsic calibration

These YAML files store the static transform from a robot's base frame
to the camera's optical frame. The tracker's `publish_camera_tf.py`
node loads one of them on launch and broadcasts the transform via
`tf2_ros.StaticTransformBroadcaster`.

## Why this lives here, not in `rl_environments`

The camera-to-robot extrinsic is a property of the **camera mount**,
not the task. Push and PnP share the same camera, so the same YAML
serves both.

## Naming convention

`<camera>_to_<robot>.yaml`. Each robot/camera pair gets one file.

Currently shipped:
- `kinect2_to_rx200.yaml`
- `kinect2_to_ned2.yaml`
- `kinect2_to_vx300s.yaml`
- `kinect2_to_ur5e.yaml`
- `zed2_to_rx200.yaml`
- `zed2_to_ned2.yaml`
- `zed2_to_vx300s.yaml`
- `zed2_to_ur5e.yaml`
- `d405_to_rx200.yaml`   (Intel RealSense D405)
- `d405_to_ned2.yaml`
- `d405_to_vx300s.yaml`
- `d405_to_ur5e.yaml`

All values are placeholders — calibrate before use. The vx300s files
mirror the RX200 mount geometry (flush on the cafe-table, camera ~0.5 m
in front + above). The UR5e files describe a different setup — the arm
sits on a ~0.59 m base with a SEPARATE cafe-table at world (0.7, 0, 0),
so the camera mounts past the table looking back; placeholder values
match the standalone-launch URDF kinect pose.

UR5e YAMLs use the BARE link name `base_link` as parent_frame because
the UR5e URDF doesn't prefix link names (unlike Interbotix
vx300s/base_link or RX200 rx200/base_link).

## File format

```yaml
parent_frame: rx200/base_link          # robot base
child_frame:  kinect2_rgb_optical_frame  # camera optical frame

translation:                            # metres
  x: 0.50
  y: 0.00
  z: 0.50

rotation_rpy_deg:                       # degrees, ROS XYZ extrinsic RPY
  roll:  -90.0
  pitch:   0.0
  yaw:   -60.0
```

The launch files pass this file via the `extrinsic_file` arg:

```bash
roslaunch rl_envs_cube_tracker kinect2.launch \
    target_frame:=rx200/base_link
# default extrinsic_file is config/extrinsics/kinect2_to_rx200.yaml

# Override to use a different mount calibration:
roslaunch rl_envs_cube_tracker kinect2.launch \
    extrinsic_file:=$(rospack find my_setup)/config/kinect2_mount_b.yaml \
    target_frame:=rx200/base_link
```

## Calibration procedures

Listed cheapest → most accurate.

### 1. Tape measure + protractor (10 minutes, ±2 cm / ±5°)

Set up a fixed camera mount. Then:

1. Measure where the camera lens centre sits **relative to the robot
   base origin** (the centre of the `rx200/base_link` flange).
2. Type `translation.x` (forward), `translation.y` (left of base), and
   `translation.z` (above base) into the YAML in metres.
3. For rotation, start with the optical-frame defaults:
   - **Looking horizontally forward (camera y-axis up):**
     `roll=-90, pitch=0, yaw=-90`
   - **Tilted ~30° down from horizontal:** add to yaw or pitch
     depending on your mount geometry.
   - **Pure top-down (camera looking straight down):**
     `roll=180, pitch=0, yaw=0`

Coarse but useful for getting first detections within ~5 cm.

### 2. RViz + manual nudging (20 minutes, ±0.5 cm / ±1°)

1. Roughly calibrate by step 1 above.
2. `roslaunch rl_envs_cube_tracker kinect2.launch
   target_frame:=rx200/base_link`.
3. Open RViz, fix frame to `rx200/base_link`, add a `PoseStamped`
   display on `/cube_pose`.
4. Place the cube at a known position (use a printed grid or measure
   from the robot base).
5. Adjust translation + rotation in the YAML until the published
   `/cube_pose` matches the measured position. `rosparam set` won't
   take effect for static TF — restart the launch after each edit.

### 3. Hand-eye calibration (1+ hour, ±1–3 mm)

Use [`easy_handeye`](https://github.com/IFL-CAMP/easy_handeye) or
similar. Mount a tag on the robot end-effector, drive the arm to ~20
poses, capture each tag detection, and the solver fits the
camera-to-robot transform. Most accurate; required for any precision
task. The output is the same parent→child transform — paste the
solved x/y/z + RPY (converted from quaternion) into the YAML.

## Optical-frame convention quick reference

For ROS standard camera frames (`*_rgb_optical_frame`,
`*_left_camera_optical_frame`):

- `+z` = viewing direction (out of the lens)
- `+x` = image-right
- `+y` = image-down

So a camera **pointing straight forward** in the world has its
optical-frame `+z` aligned with the world `+x`. That mismatch is why
the rotation values look unintuitive — you're rotating from "robot
base axes" to "optical axes".

The robot-side `link_optical_frame` URDF entry usually applies a
`-90, 0, -90` RPY to bring optical into ROS-standard convention. The
extrinsic YAML adds whatever extra rotation your mount introduces.
