cmake_minimum_required(VERSION 3.0.2)
project(min_curv_ros_wrapper)

# Use catkin_simple
find_package(catkin_simple REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(min_curv_lib REQUIRED)  # Ensure your library is available

# Initialize catkin_simple
catkin_simple(ALL)

# Add message files
cs_add_message_files(
  FILES
  Paths.msg
)

# Generate added messages and services
cs_generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

# Include directories
include_directories(include ${catkin_INCLUDE_DIRS})

# Add the header files for the ROS wrapper
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)  # Add this line

# Add the executable
cs_add_executable(ros_wrapper src/main.cpp 
                              src/ros_wrapper.cpp)

# Add dependencies to the node (e.g., message generation)
add_dependencies(ros_wrapper ${PROJECT_NAME}_generate_messages_cpp)

# Link the executable against the libraries
target_link_libraries(ros_wrapper ${catkin_LIBRARIES} min_curv_lib)

# Install targets
cs_install()

# Export the project
cs_export()

