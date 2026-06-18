#!/usr/bin/env python3
"""
Vehicle Control Arbitration Node
-----------------------------------
Subscribes to:
  - /adas/acc_cmd  (std_msgs/Float32) -- throttle/brake from ACC
  - /adas/lane_cmd (std_msgs/Float32) -- steering from LKA
  - /adas/aeb_cmd  (std_msgs/Float32) -- emergency brake flag from AEB

Publishes:
  - /carla/ego_vehicle/vehicle_control_cmd (carla_msgs/CarlaEgoVehicleControl)

Arbitration rules:
  - If AEB requests a brake, it overrides ACC throttle entirely and sets
    full brake, regardless of what ACC wants.
  - Otherwise, ACC output controls throttle/brake.
  - LKA output always controls steering (AEB/ACC don't affect steering).
"""

import rospy
from std_msgs.msg import Float32

try:
    from carla_msgs.msg import CarlaEgoVehicleControl
except ImportError:
    CarlaEgoVehicleControl = None


class VehicleControlNode:
    def __init__(self):
        rospy.init_node("vehicle_control_node")

        self.acc_cmd = 0.0
        self.lane_cmd = 0.0
        self.aeb_cmd = 0.0

        if CarlaEgoVehicleControl is not None:
            self.pub = rospy.Publisher(
                "/carla/ego_vehicle/vehicle_control_cmd",
                CarlaEgoVehicleControl, queue_size=10
            )
        else:
            rospy.logwarn("carla_msgs not found - publishing to /adas/debug_control instead")
            self.pub = rospy.Publisher("/adas/debug_control", Float32, queue_size=10)

        rospy.Subscriber("/adas/acc_cmd", Float32, self.acc_cb)
        rospy.Subscriber("/adas/lane_cmd", Float32, self.lane_cb)
        rospy.Subscriber("/adas/aeb_cmd", Float32, self.aeb_cb)

        rospy.Timer(rospy.Duration(0.05), self.publish_control)
        rospy.loginfo("vehicle_control_node started")

    def acc_cb(self, msg):
        self.acc_cmd = msg.data

    def lane_cb(self, msg):
        self.lane_cmd = msg.data

    def aeb_cb(self, msg):
        self.aeb_cmd = msg.data

    def publish_control(self, _event):
        if CarlaEgoVehicleControl is None:
            self.pub.publish(Float32(data=self.acc_cmd))
            return

        control = CarlaEgoVehicleControl()

        if self.aeb_cmd >= 1.0:
            control.throttle = 0.0
            control.brake = 1.0
        elif self.acc_cmd >= 0:
            control.throttle = float(self.acc_cmd)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = float(-self.acc_cmd)

        control.steer = float(self.lane_cmd)
        control.hand_brake = False
        control.manual_gear_shift = False

        self.pub.publish(control)


if __name__ == "__main__":
    try:
        VehicleControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
