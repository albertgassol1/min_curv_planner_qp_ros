cmake_minimum_required(VERSION 3.0.2)
project(boundary_publisher_example)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  min_curv_msgs  # Your custom message package
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs min_curv_msgs
)

# Install Python scripts
catkin_install_python(PROGRAMS
  src/boundary_publisher_example.py  # Path to your Python node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

