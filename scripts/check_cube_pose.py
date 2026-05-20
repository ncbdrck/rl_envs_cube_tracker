#!/usr/bin/env python3
"""
Subscribe to ``/cube_pose`` for ``~duration`` seconds and exit
PASS (0) / FAIL (1) based on:

- At least ``~min_count`` messages received.
- (Optional) Mean position within ``~position_tol`` of ``~expected_position``.
- (Optional) All messages stamped with ``~expected_frame_id``.

Used as the gating "checker" node in ``e2e_dry_run.launch`` —
``required="true"`` on this node makes roslaunch tear the whole rig
down once it exits.

Parameters
----------
~topic              : str, default "/cube_pose"
~duration           : float, default 5.0
~min_count          : int, default 5
~expected_position  : [x, y, z] metres, default None (skip check)
~position_tol       : float, default 0.05
~expected_frame_id  : str, default "" (skip check)
"""
from __future__ import annotations

import sys
from typing import List, Optional

import rospy
from geometry_msgs.msg import PoseStamped


class CubePoseChecker:
    def __init__(self) -> None:
        self.topic = str(rospy.get_param("~topic", "/cube_pose"))
        self.duration = float(rospy.get_param("~duration", 5.0))
        self.min_count = int(rospy.get_param("~min_count", 5))
        self.position_tol = float(rospy.get_param("~position_tol", 0.05))

        expected_pos = rospy.get_param("~expected_position", None)
        self.expected_pos: Optional[List[float]] = (
            [float(v) for v in expected_pos] if expected_pos else None
        )
        self.expected_frame = str(rospy.get_param("~expected_frame_id", ""))

        self.messages: List[PoseStamped] = []
        self.sub = rospy.Subscriber(self.topic, PoseStamped, self._cb, queue_size=50)

    def _cb(self, msg: PoseStamped) -> None:
        self.messages.append(msg)

    def run(self) -> int:
        rospy.loginfo(
            "[check_cube_pose] listening on %s for %.1fs (min=%d, tol=%.3f m)",
            self.topic, self.duration, self.min_count, self.position_tol,
        )
        rospy.sleep(self.duration)

        n = len(self.messages)
        rospy.loginfo("[check_cube_pose] received %d message(s)", n)

        failures = []

        if n < self.min_count:
            failures.append(f"count {n} < min_count {self.min_count}")

        if n > 0 and self.expected_pos is not None:
            xs = [m.pose.position.x for m in self.messages]
            ys = [m.pose.position.y for m in self.messages]
            zs = [m.pose.position.z for m in self.messages]
            mean = (sum(xs) / n, sum(ys) / n, sum(zs) / n)
            err = max(abs(mean[i] - self.expected_pos[i]) for i in range(3))
            rospy.loginfo(
                "[check_cube_pose] mean=(%.4f, %.4f, %.4f) expected=(%.4f, %.4f, %.4f) max_err=%.4f m",
                *mean, *self.expected_pos, err,
            )
            if err > self.position_tol:
                failures.append(f"position max_err {err:.4f} > tol {self.position_tol}")

        if n > 0 and self.expected_frame:
            frames = set(m.header.frame_id for m in self.messages)
            rospy.loginfo("[check_cube_pose] frame_id(s) seen: %s", sorted(frames))
            if frames != {self.expected_frame}:
                failures.append(
                    f"expected frame_id={self.expected_frame!r}, saw {sorted(frames)}"
                )

        if failures:
            for f in failures:
                rospy.logerr("[check_cube_pose] FAIL: %s", f)
            return 1

        rospy.loginfo("[check_cube_pose] PASS")
        return 0


def main() -> int:
    rospy.init_node("check_cube_pose")
    code = CubePoseChecker().run()
    return code


if __name__ == "__main__":
    sys.exit(main())
