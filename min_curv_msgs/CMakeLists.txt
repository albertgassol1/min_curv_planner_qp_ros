cmake_minimum_required(VERSION 3.0.2)
project(min_curv_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation 
                                        roscpp
                                        rospy
                                        std_msgs
                                        nav_msgs)

add_message_files(FILES Paths.msg)

generate_messages(DEPENDENCIES nav_msgs)

# Needed to generate custom messages
catkin_package(CATKIN_DEPENDS message_runtime nav_msgs)

include_directories(${catkin_INCLUDE_DIRS})