#!/usr/bin/env python3
"""
Bridge ``apriltag_ros`` detections to ``/cube_pose`` for the
``rl_environments`` real-side push / PnP envs.

Subscribes
----------
~detections : apriltag_ros/AprilTagDetectionArray
    Typically remapped from ``/tag_detections``.

Publishes
---------
~cube_pose : geometry_msgs/PoseStamped
    Typically remapped to ``/cube_pose`` so the env's PoseStamped
    subscriber picks it up directly.

Parameters
----------
~cube_tag_id : int, default 0
    AprilTag ID stuck on the cube.
~target_frame : str, default ""
    TF frame to transform the detection into before publishing
    (e.g. ``rx200/base_link``). The downstream ``rl_environments``
    consumer treats ``/cube_pose`` as a robot base-frame pose, so
    leaving this empty silently mis-localises the cube. The node
    refuses to start when empty unless ``~allow_source_frame`` is set
    (in which case the detection is republished in the camera optical
    frame and the consumer must handle the frame mismatch itself).
~allow_source_frame : bool, default False
    Explicit opt-in for publishing the detection unchanged in its
    source frame. Use only when the downstream pipeline knows the
    pose is camera-frame, not robot-frame.
~tf_timeout : float, default 1.0
    Max wait time when looking up the TF transform.
~rate_limit_hz : float, default 0.0
    If > 0, throttle publication to this rate. Useful if the camera
    runs at 30 Hz but the env only needs 10 Hz pose updates.
~max_stale_s : float, default 0.5
    Drop detections whose ``header.stamp`` is older than this many
    seconds at receipt time. Catches a stuck upstream that keeps
    re-publishing the same frozen detection (rate-limit alone uses
    receipt time, not the stamp, so it cannot detect this).

Notes
-----
* The env falls back to ``cube_init_pos`` if no message arrives within
  ``cube_pose_timeout_s`` (default 1.0 s), so a dropped frame or two
  is harmless — just don't let the publisher die silently.
"""
from __future__ import annotations

import sys

import rospy
import tf2_ros

# tf2_geometry_msgs registers do_transform_pose for PoseStamped.
import tf2_geometry_msgs  # noqa: F401

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped


class TagToCubePose:
    def __init__(self) -> None:
        self.cube_tag_id = int(rospy.get_param("~cube_tag_id", 0))
        self.target_frame = str(rospy.get_param("~target_frame", "")).strip()
        self.allow_source_frame = bool(rospy.get_param("~allow_source_frame", False))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 1.0))
        self.rate_limit_hz = float(rospy.get_param("~rate_limit_hz", 0.0))
        self.max_stale_s = float(rospy.get_param("~max_stale_s", 0.5))

        if not self.target_frame and not self.allow_source_frame:
            msg = (
                "[tag_to_cube_pose] ~target_frame is empty. The "
                "rl_environments consumer treats /cube_pose as a robot "
                "base-frame pose; publishing the source (camera optical) "
                "frame would silently mis-localise the cube. Set "
                "~target_frame (e.g. rx200/base_link), or pass "
                "~allow_source_frame:=true to acknowledge that the "
                "downstream pipeline handles the frame mismatch itself."
            )
            rospy.logfatal(msg)
            raise rospy.ROSInitException(msg)

        if self.target_frame:
            self.tf_buf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        else:
            self.tf_buf = None
            self.tf_listener = None
            rospy.logwarn(
                "[tag_to_cube_pose] ~allow_source_frame=true; republishing "
                "the detection in its source frame. The rl_environments "
                "consumer expects a robot base-frame pose, so the "
                "downstream pipeline must handle the frame mismatch."
            )

        self._min_period = (1.0 / self.rate_limit_hz) if self.rate_limit_hz > 0 else 0.0
        self._last_pub_t = 0.0

        self.pub = rospy.Publisher("~cube_pose", PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber(
            "~detections", AprilTagDetectionArray, self._on_detections, queue_size=1
        )

        rospy.loginfo(
            "[tag_to_cube_pose] cube_tag_id=%d target_frame=%s "
            "rate_limit_hz=%.1f max_stale_s=%.2f",
            self.cube_tag_id,
            self.target_frame if self.target_frame else "<source>",
            self.rate_limit_hz,
            self.max_stale_s,
        )

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        # apriltag_ros bundles tag IDs as tuples (single tags arrive as
        # length-1 tuples; bundles as length-N). We treat any detection
        # whose id-tuple contains our cube tag as a match.
        det = next(
            (d for d in msg.detections if self.cube_tag_id in tuple(d.id)),
            None,
        )
        if det is None:
            return

        # Stamp-based staleness check. Catches a stuck upstream that
        # keeps re-publishing the same frozen detection — rate-limit
        # alone uses receipt time and would let that through.
        if self.max_stale_s > 0.0 and not det.pose.header.stamp.is_zero():
            age = (rospy.Time.now() - det.pose.header.stamp).to_sec()
            if age > self.max_stale_s:
                rospy.logwarn_throttle(
                    2.0,
                    f"[tag_to_cube_pose] Dropping detection {age:.2f}s old "
                    f"(>{self.max_stale_s:.2f}s); check the camera/apriltag pipeline.",
                )
                return

        if self._min_period > 0.0:
            now = rospy.get_time()
            if (now - self._last_pub_t) < self._min_period:
                return
            self._last_pub_t = now

        # apriltag_ros nests the pose in PoseWithCovarianceStamped; we
        # only need the PoseStamped view.
        src = PoseStamped()
        src.header = det.pose.header
        src.pose = det.pose.pose.pose

        if self.tf_buf is None:
            self.pub.publish(src)
            return

        try:
            tf = self.tf_buf.lookup_transform(
                self.target_frame,
                src.header.frame_id,
                src.header.stamp,
                rospy.Duration(self.tf_timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                2.0,
                f"[tag_to_cube_pose] TF {src.header.frame_id} → "
                f"{self.target_frame} not available: {exc}",
            )
            return

        out = tf2_geometry_msgs.do_transform_pose(src, tf)
        self.pub.publish(out)


def main() -> int:
    rospy.init_node("tag_to_cube_pose")
    TagToCubePose()
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
