#pragma once
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <min_curv_ros_wrapper/Paths.h> 

#include "base_cubic_spline.hpp"
#include "curv_min.hpp"

namespace min_curv_ros_wrapper {

class RosWrapper {
public:
    RosWrapper(ros::NodeHandle& nh);
    
    // Callback functions for subscribers
    void boundariesCallback(const min_curv_ros_wrapper::Paths::ConstPtr& msg);

    // Publish results (optimized path and curvatures)
    void publishOptimizedPath(const std::vector<Eigen::Vector2d>& opt_points,
                              const std::vector<double>& init_curv,
                              const std::vector<double>& opt_curv) const;

private:
    ros::NodeHandle nh_;

    // Subscribers
    ros::Subscriber boundaries_sub_;

    // Publishers
    struct Publishers {
        ros::Publisher optimized_path_pub_;
        ros::Publisher initial_curvature_pub_;
        ros::Publisher optimized_curvature_pub_;
    } pub_;

    // Internal data storage for boundaries and centerline splines
    std::shared_ptr<spline::BaseCubicSpline> centerline_spline_;
    std::shared_ptr<spline::BaseCubicSpline> left_boundary_spline_;
    std::shared_ptr<spline::BaseCubicSpline> right_boundary_spline_;
    std::shared_ptr<spline::BaseCubicSpline> optimized_trajectory_;

    // Solver pointer
    std::unique_ptr<spline::optimization::MinCurvatureOptimizer> optimizer_;

    // Function to process the data and run the optimizer
    void optimizeTrajectory();
};

} // namespace min_curv_ros_wrapper
