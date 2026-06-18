#!/usr/bin/env python3
"""
Adaptive Cruise Control (ACC) Node
-----------------------------------
Subscribes to:
  - /adas/obstacle_info (adas_msgs/ObstacleInfo)
  - /carla/ego_vehicle/vehicle_status (carla_msgs/CarlaEgoVehicleStatus)

Publishes:
  - /adas/acc_cmd (std_msgs/Float32) -- desired throttle in [-1, 1]
                                         (negative = braking)

Maintains a target speed when the road ahead is clear, and backs off to
maintain a safe following distance when a slower vehicle is detected.
"""

import rospy
from std_msgs.msg import Float32
from adas_msgs.msg import ObstacleInfo

try:
    from carla_msgs.msg import CarlaEgoVehicleStatus
except ImportError:
    CarlaEgoVehicleStatus = None


class ACCNode:
    def __init__(self):
        rospy.init_node("acc_node")

        self.target_speed = rospy.get_param("~target_speed", 15.0)    # m/s
        self.safe_distance = rospy.get_param("~safe_distance", 10.0)  # m
        self.kp_speed = 0.5
        self.kp_distance = 0.3

        self.current_speed = 0.0
        self.obstacle = None

        self.pub = rospy.Publisher("/adas/acc_cmd", Float32, queue_size=10)
        rospy.Subscriber("/adas/obstacle_info", ObstacleInfo, self.obstacle_cb)

        if CarlaEgoVehicleStatus is not None:
            rospy.Subscriber("/carla/ego_vehicle/vehicle_status",
                              CarlaEgoVehicleStatus, self.status_cb)
        else:
            rospy.logwarn("carla_msgs not found - vehicle speed feedback disabled")

        rospy.Timer(rospy.Duration(0.1), self.control_loop)
        rospy.loginfo("acc_node started (target_speed=%.1f m/s, safe_distance=%.1f m)",
                       self.target_speed, self.safe_distance)

    def status_cb(self, msg):
        self.current_speed = msg.velocity  # m/s

    def obstacle_cb(self, msg):
        self.obstacle = msg

    def control_loop(self, _event):
        speed_error = self.target_speed - self.current_speed
        throttle = self.kp_speed * speed_error

        if self.obstacle and self.obstacle.obstacle_detected:
            distance_error = self.obstacle.distance - self.safe_distance
            if distance_error < 0:
                # Too close: blend in a braking term proportional to how
                # far inside the safe distance we are.
                brake = self.kp_distance * distance_error  # negative
                throttle = min(throttle, brake)

        throttle = max(-1.0, min(1.0, throttle))
        self.pub.publish(Float32(data=throttle))


if __name__ == "__main__":
    try:
        ACCNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
