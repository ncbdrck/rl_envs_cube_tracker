#!/usr/bin/env python3
"""
Broadcast a static TF from a robot base frame to a camera optical frame,
loaded from a YAML extrinsic file.

Parameters
----------
~extrinsic_file : str (required)
    Absolute path to the extrinsic YAML. See
    rl_envs_cube_tracker/config/extrinsics/README.md for format +
    calibration procedures.

Topics
------
Publishes on /tf_static via tf2_ros.StaticTransformBroadcaster.

Notes
-----
* RPY is in **degrees** in the YAML for user-friendliness; we convert
  to radians for tf.transformations.quaternion_from_euler.
* If your camera driver already publishes the camera_optical → robot
  base TF (e.g. via a URDF mount + robot_state_publisher), DO NOT also
  launch this node — you'll get two static transforms for the same
  pair and TF lookups become non-deterministic.
"""
from __future__ import annotations

import math
import sys

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler


def _load_extrinsic(path: str) -> dict:
    with open(path, "r") as fh:
        cfg = yaml.safe_load(fh)
    for key in ("parent_frame", "child_frame", "translation", "rotation_rpy_deg"):
        if key not in cfg:
            raise KeyError(f"Extrinsic file {path} missing required key '{key}'")
    return cfg


def main() -> int:
    rospy.init_node("publish_camera_tf")

    path = rospy.get_param("~extrinsic_file", None)
    if not path:
        rospy.logfatal("[publish_camera_tf] ~extrinsic_file param is required")
        return 1

    try:
        cfg = _load_extrinsic(path)
    except (OSError, yaml.YAMLError, KeyError) as exc:
        rospy.logfatal(f"[publish_camera_tf] failed to load {path}: {exc}")
        return 1

    t = cfg["translation"]
    r = cfg["rotation_rpy_deg"]

    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = cfg["parent_frame"]
    msg.child_frame_id = cfg["child_frame"]
    msg.transform.translation.x = float(t["x"])
    msg.transform.translation.y = float(t["y"])
    msg.transform.translation.z = float(t["z"])

    qx, qy, qz, qw = quaternion_from_euler(
        math.radians(float(r["roll"])),
        math.radians(float(r["pitch"])),
        math.radians(float(r["yaw"])),
    )
    msg.transform.rotation.x = qx
    msg.transform.rotation.y = qy
    msg.transform.rotation.z = qz
    msg.transform.rotation.w = qw

    br = tf2_ros.StaticTransformBroadcaster()
    br.sendTransform(msg)
    rospy.loginfo(
        "[publish_camera_tf] %s → %s | xyz=(%.3f, %.3f, %.3f) rpy_deg=(%.1f, %.1f, %.1f)",
        cfg["parent_frame"],
        cfg["child_frame"],
        msg.transform.translation.x,
        msg.transform.translation.y,
        msg.transform.translation.z,
        r["roll"], r["pitch"], r["yaw"],
    )
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
