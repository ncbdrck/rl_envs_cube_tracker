#!/usr/bin/env python3
"""
Publish a fake ``apriltag_ros/AprilTagDetectionArray`` on
``/tag_detections`` for end-to-end dry-running of the cube tracker
*without* a camera or robot.

Useful for:
- Verifying the adapter (``tag_to_cube_pose.py``) republishes correctly.
- Stress-testing the env's freshness fallback (kill this mid-run).
- Smoke-testing the TF chain with ``target_frame:=rx200/base_link``.

Parameters
----------
~tag_id     : int, default 0
    Tag ID. The adapter's ``cube_tag_id`` must match (default 0).
~tag_size   : float, default 0.030
    Edge length in metres (cosmetic — apriltag_ros uses it for pose
    estimation but our adapter ignores it).
~frame_id   : str, default "kinect2_rgb_optical_frame"
    Header frame for the fake detection.
~hz         : float, default 10.0
~position   : [x, y, z] metres, default [0.25, 0.0, 0.50]
~orientation: [x, y, z, w] quat, default [0, 0, 0, 1]
"""
from __future__ import annotations

import sys

import rospy
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray


def main() -> int:
    rospy.init_node("fake_tag_publisher")

    tag_id = int(rospy.get_param("~tag_id", 0))
    tag_size = float(rospy.get_param("~tag_size", 0.030))
    frame_id = str(rospy.get_param("~frame_id", "kinect2_rgb_optical_frame"))
    hz = float(rospy.get_param("~hz", 10.0))
    pos = list(rospy.get_param("~position", [0.25, 0.0, 0.50]))
    quat = list(rospy.get_param("~orientation", [0.0, 0.0, 0.0, 1.0]))

    if len(pos) != 3:
        raise ValueError(f"~position must be length 3, got {pos!r}")
    if len(quat) != 4:
        raise ValueError(f"~orientation must be length 4, got {quat!r}")

    pub = rospy.Publisher("/tag_detections", AprilTagDetectionArray, queue_size=1)
    rate = rospy.Rate(hz)

    rospy.loginfo(
        "[fake_tag_publisher] tag_id=%d frame_id=%s hz=%.1f pos=%s quat=%s",
        tag_id, frame_id, hz, pos, quat,
    )

    while not rospy.is_shutdown():
        arr = AprilTagDetectionArray()
        arr.header.stamp = rospy.Time.now()
        arr.header.frame_id = frame_id

        det = AprilTagDetection()
        det.id = [tag_id]
        det.size = [tag_size]
        det.pose.header.stamp = arr.header.stamp
        det.pose.header.frame_id = frame_id
        det.pose.pose.pose.position.x = float(pos[0])
        det.pose.pose.pose.position.y = float(pos[1])
        det.pose.pose.pose.position.z = float(pos[2])
        det.pose.pose.pose.orientation.x = float(quat[0])
        det.pose.pose.pose.orientation.y = float(quat[1])
        det.pose.pose.pose.orientation.z = float(quat[2])
        det.pose.pose.pose.orientation.w = float(quat[3])
        arr.detections.append(det)

        pub.publish(arr)
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break

    return 0


if __name__ == "__main__":
    sys.exit(main())
