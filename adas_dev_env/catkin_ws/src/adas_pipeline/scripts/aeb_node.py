#!/usr/bin/env python3
"""
Automatic Emergency Braking (AEB) Node
-----------------------------------------
Subscribes to:
  - /adas/obstacle_info (adas_msgs/ObstacleInfo)

Publishes:
  - /adas/aeb_cmd (std_msgs/Float32) -- 0.0 = no intervention,
                                          1.0 = full emergency brake

Computes time-to-collision (TTC) from obstacle distance and relative
velocity. If TTC drops below a threshold, issues a full-brake command
that overrides all other modules in vehicle_control_node.
"""

import rospy
from std_msgs.msg import Float32
from adas_msgs.msg import ObstacleInfo


class AEBNode:
    def __init__(self):
        rospy.init_node("aeb_node")

        self.ttc_threshold = rospy.get_param("~ttc_threshold", 1.5)  # seconds

        self.pub = rospy.Publisher("/adas/aeb_cmd", Float32, queue_size=10)
        rospy.Subscriber("/adas/obstacle_info", ObstacleInfo, self.obstacle_cb)

        rospy.loginfo("aeb_node started (ttc_threshold=%.2fs)", self.ttc_threshold)

    def obstacle_cb(self, msg):
        brake_cmd = 0.0

        if msg.obstacle_detected and msg.relative_velocity < 0:
            # relative_velocity < 0 means the gap is closing
            ttc = msg.distance / abs(msg.relative_velocity)
            if ttc < self.ttc_threshold:
                brake_cmd = 1.0
                rospy.logwarn("AEB triggered! TTC=%.2fs, distance=%.2fm",
                               ttc, msg.distance)

        self.pub.publish(Float32(data=brake_cmd))


if __name__ == "__main__":
    try:
        AEBNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
