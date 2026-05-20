# rl_envs_cube_tracker

Real-side cube pose publisher for the `rl_environments` push / pick-and-place
task envs. Wraps [`apriltag_ros`](https://github.com/AprilRobotics/apriltag_ros)
and republishes the detected cube tag as a `geometry_msgs/PoseStamped` on
`/cube_pose` — the topic the env subscribes to.

Robot-agnostic. Same package works for RX200, NED2, UR5e — just point it at
whichever camera you're using and (optionally) the robot's base frame.

## What it does

```
  camera image  ─►  apriltag_ros  ─►  /tag_detections  ─►  tag_to_cube_pose
                  (continuous_detection)                      │
                                                              ▼
                                                          /cube_pose
                                                          (PoseStamped)
                                                              │
                                                              ▼
                                              rl_environments real envs
                                              (RX200{Push,PnP}Real-v0,
                                               NED2 + UR5e analogues later)
```

The adapter (`tag_to_cube_pose.py`) picks the cube's tag ID out of the
detection batch, optionally TF-transforms it into the robot's base
frame, and publishes a clean `PoseStamped` on `/cube_pose`.

## Default tag

| Property | Value |
|---|---|
| Family | `tag36h11` |
| ID | `0` |
| Border size | `30 mm` |
| Recommended print | ~50 mm sticker (30 mm tag + 10 mm white quiet zone) |

Generate at <https://chev.me/arucogen/> (set Dictionary to `36h11`, size `30 mm`)
or with the `apriltag-generate-image` CLI:

```bash
apriltag-generate-image -f tag36h11 -d 1 0
# then scale to 30 mm in your favourite layout tool and print
```

Stick it on the top face of the cube. The detection pose origin sits at
the **tag centre**, with z pointing out of the tag face.

## Prerequisites

### ROS packages

```bash
# AprilTag detector (the heavy lifting — detection + pose estimation)
sudo apt install ros-noetic-apriltag-ros

# tf2 helpers used by the adapter (usually pre-installed with ROS desktop)
sudo apt install ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs
```

Or, from inside the workspace, let `rosdep` resolve everything declared
in `package.xml`:

```bash
cd ~/rl_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Camera driver (provided by you)

The launch files assume the appropriate camera driver is already
publishing rectified images + camera_info on the expected namespace:

- **kinect2.launch** expects `/kinect2/qhd/image_color_rect` +
  `/kinect2/qhd/camera_info` (default `iai_kinect2 / kinect2_bridge`
  layout).
- **zed2.launch** expects `/zed2/zed_node/rgb/image_rect_color` +
  `/zed2/zed_node/rgb/camera_info` (default `zed_wrapper` layout).

Both can be overridden via the `camera_ns` / `image_topic` launch args.

## Quick start

### 1. Build

```bash
cd ~/rl_ws
catkin_make
source devel/setup.bash
```

### 2. Launch the pipeline

Kinect2 (assumes `kinect2_bridge` already running on `/kinect2`):

```bash
roslaunch rl_envs_cube_tracker kinect2.launch
```

ZED2 (assumes `zed_wrapper` already running on `/zed2/zed_node`):

```bash
roslaunch rl_envs_cube_tracker zed2.launch
```

Already have your own apriltag_ros / detector publishing `/tag_detections`?
Skip straight to the adapter:

```bash
roslaunch rl_envs_cube_tracker adapter_only.launch
```

### 3. Verify

```bash
rostopic echo /cube_pose
```

You should see a `PoseStamped` stream at the detection rate.

## TF-transforming into the robot base

By default, `/cube_pose` is in the **camera optical frame**. The env handles
both — but for downstream consumers (RViz, debug tools, log analysis) it's
convenient to publish in the robot base frame:

```bash
roslaunch rl_envs_cube_tracker kinect2.launch \
    target_frame:=rx200/base_link
```

This requires the `camera_optical → rx200/base_link` TF chain to be
populated. The launch files include a small TF broadcaster
(`publish_camera_tf.py`) that loads the camera extrinsic from a YAML
in `config/extrinsics/` and publishes it on `/tf_static`. **You must
calibrate that YAML for your camera mount before the published
`/cube_pose` will be accurate** — the shipped values are placeholders.

Calibration procedures (tape measure, RViz nudging, hand-eye) are in
[`config/extrinsics/README.md`](config/extrinsics/README.md).

If your robot URDF / `robot_state_publisher` already provides the
camera_optical → base_link TF, suppress the broadcaster:

```bash
roslaunch rl_envs_cube_tracker kinect2.launch \
    target_frame:=rx200/base_link \
    publish_extrinsic_tf:=false
```

To point at a different extrinsic file (e.g. a second mount calibration):

```bash
roslaunch rl_envs_cube_tracker kinect2.launch \
    target_frame:=rx200/base_link \
    extrinsic_file:=$(rospack find my_setup)/config/kinect2_mount_b.yaml
```

## All launch args

Common to all launches:

| Arg | Default | Purpose |
|---|---|---|
| `cube_tag_id` | `0` | AprilTag ID stuck on the cube. |
| `target_frame` | `""` | If non-empty, TF-transform the pose into this frame. |
| `output_topic` | `/cube_pose` | Where the env subscribes. |
| `rate_limit_hz` | `0.0` | If > 0, throttle to this rate. Default = no throttle. |
| `publish_extrinsic_tf` | `true` | Spawn the camera TF broadcaster. Disable if your URDF already publishes the chain. |
| `extrinsic_file` | `config/extrinsics/<camera>_to_rx200.yaml` | Calibration YAML loaded by the broadcaster. |

`kinect2.launch` / `zed2.launch` also accept `camera_ns` + `image_topic`
overrides if your bringup uses a non-default base name.

## Multiple cubes

Add entries to `config/tags.yaml`:

```yaml
standalone_tags:
  [
    {id: 0, size: 0.030, name: cube},
    {id: 1, size: 0.030, name: cube_b},
  ]
```

Then run two adapters in different namespaces — each picks one ID via
`~cube_tag_id` and publishes to a distinct topic.

## Files

```
rl_envs_cube_tracker/
├── package.xml
├── CMakeLists.txt
├── launch/
│   ├── kinect2.launch        # apriltag_ros + adapter + TF, kinect2 topics
│   ├── zed2.launch           # apriltag_ros + adapter + TF, zed2 topics
│   ├── adapter_only.launch   # adapter alone, BYO detector
│   └── e2e_dry_run.launch    # headless self-test: fake → adapter → check
├── config/
│   ├── tags.yaml             # tag IDs + sizes (default: id 0, 30 mm)
│   ├── settings.yaml         # apriltag detector params (tag36h11)
│   └── extrinsics/           # per-camera-per-robot static TF YAMLs
│       ├── kinect2_to_rx200.yaml
│       ├── zed2_to_rx200.yaml
│       └── README.md         # calibration procedures
└── scripts/
    ├── tag_to_cube_pose.py   # AprilTagDetectionArray → PoseStamped
    ├── publish_camera_tf.py  # loads extrinsic YAML → /tf_static
    ├── fake_tag_publisher.py # dry-run only: synthetic detections
    └── check_cube_pose.py    # dry-run only: subscribe + PASS/FAIL gate
```

## End-to-end dry-run (no camera, no robot)

Verify the whole adapter + TF chain works before plugging in hardware.
Two terminals:

```bash
# 1. roscore
roscore

# 2. Dry-run (no TF — verifies adapter pass-through in camera frame)
roslaunch rl_envs_cube_tracker e2e_dry_run.launch

# 2b. With TF chain (verifies the camera_optical → robot_base transform)
roslaunch rl_envs_cube_tracker e2e_dry_run.launch \
    target_frame:=rx200/base_link publish_extrinsic_tf:=true
```

The launch:

- `fake_tag_publisher.py` — emits a fake `AprilTagDetectionArray` on
  `/tag_detections` at 20 Hz (no camera or apriltag_ros needed).
- `tag_to_cube_pose.py` — adapter under test.
- `publish_camera_tf.py` — when enabled, spawns the static TF.
- `check_cube_pose.py` — subscribes to `/cube_pose` for `duration` s,
  prints `PASS` / `FAIL`, then exits. `required="true"` makes
  roslaunch tear the whole rig down on exit.

Useful args: `duration` (s), `hz`, `min_count`, `fake_position`,
`target_frame`, `publish_extrinsic_tf`.

Use this as a regression test after touching `tag_to_cube_pose.py` or
the extrinsic YAMLs.

## Sim?

You don't need this in sim. The sim envs read cube pose directly from
Gazebo's `/gazebo/get_model_state` service (faster, noise-free). The
`/cube_pose` contract is real-only — same observation shape downstream,
different upstream pipeline.

## Contact

[j.kapukotuwa@research.ait.ie](mailto:j.kapukotuwa@research.ait.ie)
