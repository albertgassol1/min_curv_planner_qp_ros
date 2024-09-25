# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project Name and Version
project(min_curv_planner_qp VERSION 1.0)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add subdirectories for the library and examples
add_subdirectory(min_curv_lib)
add_subdirectory(examples)
