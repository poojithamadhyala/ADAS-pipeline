cmake_minimum_required(VERSION 3.0.2)
project(adas_pipeline)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  adas_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/sensor_fusion_node.py
  scripts/acc_node.py
  scripts/lane_keeping_node.py
  scripts/aeb_node.py
  scripts/vehicle_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
