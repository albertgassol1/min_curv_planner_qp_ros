cmake_minimum_required(VERSION 3.0.2)
project(min_curv_ros_wrapper)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_definitions(-Wall -Werror)

# Use catkin_simple
find_package (Eigen3 3.3 REQUIRED)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  include
  /usr/local/include/osqp/
  ${catkin_INCLUDE_DIRS})

cs_add_library(${PROJECT_NAME} src/ros_wrapper.cpp 
                               src/main.cpp)

cs_add_executable(${PROJECT_NAME}_exec src/ros_wrapper.cpp
                                       src/main.cpp)

target_link_libraries(${PROJECT_NAME}_exec ${PROJECT_NAME}
                                           ${catkin_LIBRARIES}
                                           osqp::osqp
                                           OsqpEigen::OsqpEigen
                                           Eigen3::Eigen)

cs_export()
