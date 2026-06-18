#!/usr/bin/env python3
"""
Sensor Fusion Node
-------------------
Subscribes to:
  - /carla/ego_vehicle/radar_front              (sensor_msgs/PointCloud2)
  - /carla/ego_vehicle/camera_front/image_color (sensor_msgs/Image)

Publishes:
  - /adas/obstacle_info (adas_msgs/ObstacleInfo)

Combines a simplified radar-based obstacle distance/velocity estimate
with a simple OpenCV lane-detection pipeline run on the front camera
image, and republishes a single fused ObstacleInfo message that
downstream ADAS modules (ACC, AEB, Lane Keeping) consume.
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
from adas_msgs.msg import ObstacleInfo


class SensorFusionNode:
    def __init__(self):
        rospy.init_node("sensor_fusion_node")

        self.bridge = CvBridge()
        self.latest_lane_offset = 0.0
        self.latest_heading_error = 0.0

        self.pub = rospy.Publisher("/adas/obstacle_info", ObstacleInfo, queue_size=10)

        rospy.Subscriber("/carla/ego_vehicle/radar_front", PointCloud2, self.radar_cb)
        rospy.Subscriber("/carla/ego_vehicle/camera_front/image_color", Image, self.camera_cb)

        rospy.loginfo("sensor_fusion_node started")

    def camera_cb(self, msg):
        """Run a simple Canny + Hough lane detector and estimate lane offset."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("cv_bridge conversion failed: %s", e)
            return

        offset, heading_err = self._estimate_lane_geometry(frame)
        self.latest_lane_offset = offset
        self.latest_heading_error = heading_err

    def _estimate_lane_geometry(self, frame):
        h, w = frame.shape[:2]
        roi = frame[int(h * 0.6):h, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, threshold=40,
            minLineLength=30, maxLineGap=20
        )

        if lines is None:
            return self.latest_lane_offset, self.latest_heading_error

        left_xs, right_xs, slopes = [], [], []
        mid_x = w / 2.0

        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) < 0.3:
                continue
            slopes.append(slope)
            avg_x = (x1 + x2) / 2.0
            if avg_x < mid_x:
                left_xs.append(avg_x)
            else:
                right_xs.append(avg_x)

        if not left_xs or not right_xs:
            return self.latest_lane_offset, self.latest_heading_error

        lane_center = (np.mean(left_xs) + np.mean(right_xs)) / 2.0
        pixel_offset = lane_center - mid_x

        # Rough pixel-to-meter conversion (assumes ~3.5m lane width spans
        # the detected left/right marking span in the ROI)
        lane_px_width = max(np.mean(right_xs) - np.mean(left_xs), 1.0)
        meters_per_pixel = 3.5 / lane_px_width
        lane_offset = pixel_offset * meters_per_pixel

        heading_error = float(np.mean(slopes))

        return float(lane_offset), heading_error

    def radar_cb(self, msg):
        """Find the closest point ahead and estimate distance/closing speed."""
        points = list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z", "Velocity"), skip_nans=True
        ))

        info = ObstacleInfo()
        info.lane_offset = self.latest_lane_offset
        info.heading_error = self.latest_heading_error

        if not points:
            info.obstacle_detected = False
            info.distance = float("inf")
            info.relative_velocity = 0.0
            self.pub.publish(info)
            return

        # CARLA radar points: x = forward distance, Velocity = relative speed
        closest = min(points, key=lambda p: p[0])
        info.distance = float(closest[0])
        info.relative_velocity = float(closest[3])
        info.obstacle_detected = True

        self.pub.publish(info)


if __name__ == "__main__":
    try:
        SensorFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
