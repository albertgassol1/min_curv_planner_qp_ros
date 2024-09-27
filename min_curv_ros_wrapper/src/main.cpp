// main.cpp
#include <ros/ros.h>
#include "min_curv_ros_wrapper/ros_wrapper.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "min_curv_ros_wrapper_node");
    ros::NodeHandle nh;

    min_curv_ros_wrapper::RosWrapper ros_wrapper(nh);

    ros::spin();  // Keep the node alive and responsive to callbacks

    return 0;
}
