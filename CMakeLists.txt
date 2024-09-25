cmake_minimum_required(VERSION 3.10)
project(BSplineCurvature)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 3.3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Boost 1.65 REQUIRED COMPONENTS program_options)

# Include directories for Eigen and OsqpEigen
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OsqpEigen_INCLUDE_DIRS})
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "Boost not found. Please install Boost and try again.")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)

# Add executable in bin directory
add_executable(min_curvature_spline main.cpp 
                                    cubic_b_spline.cpp 
                                    cubic_spline.cpp 
                                    base_cubic_spline.cpp 
                                    curv_min.cpp
                                    )

target_link_libraries(min_curvature_spline 
                      Eigen3::Eigen
                      OsqpEigen::OsqpEigen
                      ${Boost_LIBRARIES}
                      )

target_include_directories(min_curvature_spline PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

