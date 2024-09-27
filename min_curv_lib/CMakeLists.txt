cmake_minimum_required(VERSION 3.0.2)
# Set the library name
project(min_curv_lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_definitions(-Wall -Werror)

set(CMAKE_BUILD_TYPE Release)

find_package (Eigen3 3.3 REQUIRED)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(include
                    /usr/local/include/osqp/
                    ${catkin_INCLUDE_DIRS})

                    
cs_add_library(${PROJECT_NAME} src/base_cubic_spline.cpp 
                               src/cubic_b_spline.cpp
                               src/cubic_spline.cpp
                               src/curv_min.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      osqp::osqp
                                      OsqpEigen::OsqpEigen
                                      Eigen3::Eigen)

cs_export()
