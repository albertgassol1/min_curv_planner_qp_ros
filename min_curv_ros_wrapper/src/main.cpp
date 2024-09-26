// main.cpp
#include <ros/ros.h>
#include "min_curv_ros_wrapper/ros_wrapper.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "curvature_optimizer");
    ros::NodeHandle nh;

    RosWrapper ros_wrapper(nh);

    ros::spin();  // Keep the node alive and responsive to callbacks

    return 0;
}
