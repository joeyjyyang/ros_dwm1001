cmake_minimum_required(VERSION 2.8.3)
project(ros_dwm1001)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  sensor_msgs
  visualization_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  Anchor.msg
  Tag.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime sensor_msgs visualization_msgs
)

catkin_install_python(
   PROGRAMS scripts/dwm1001_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

