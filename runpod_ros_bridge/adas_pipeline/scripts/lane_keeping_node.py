#!/usr/bin/env python3
"""
Lane Keeping Assist (LKA) Node
--------------------------------
Subscribes to:
  - /adas/obstacle_info (adas_msgs/ObstacleInfo)
    -- uses the lane_offset and heading_error fields produced by the
       sensor fusion node's camera-based lane detector

Publishes:
  - /adas/lane_cmd (std_msgs/Float32) -- steering correction in [-1, 1]
                                          (negative = steer left)

Simple PD controller on lane offset + heading error to keep the vehicle
centered in its lane.
"""

import rospy
from std_msgs.msg import Float32
from adas_msgs.msg import ObstacleInfo


class LaneKeepingNode:
    def __init__(self):
        rospy.init_node("lane_keeping_node")

        self.kp_offset = 0.08
        self.kd_heading = 0.5

        self.pub = rospy.Publisher("/adas/lane_cmd", Float32, queue_size=10)
        rospy.Subscriber("/adas/obstacle_info", ObstacleInfo, self.obstacle_cb)

        rospy.loginfo("lane_keeping_node started")

    def obstacle_cb(self, msg):
        offset = msg.lane_offset
        heading = msg.heading_error

        steering = -(self.kp_offset * offset + self.kd_heading * heading)
        steering = max(-1.0, min(1.0, steering))

        self.pub.publish(Float32(data=steering))


if __name__ == "__main__":
    try:
        LaneKeepingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
