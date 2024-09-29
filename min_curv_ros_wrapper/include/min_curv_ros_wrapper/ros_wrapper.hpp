#pragma once
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>

#include "min_curv_msgs/Paths.h" 
#include "min_curv_lib/base_cubic_spline.hpp"
#include "min_curv_lib/cubic_spline.hpp"
#include "min_curv_lib/cubic_b_spline.hpp"
#include "min_curv_lib/curv_min.hpp"

namespace min_curv_ros_wrapper {

class RosWrapper {
public:
    RosWrapper(ros::NodeHandle& nh);
    
    // Callback functions for subscribers
    void boundariesCallback(const min_curv_msgs::Paths::ConstPtr& msg);

    // Publish results (optimized path and curvatures)
    void publish(const std::vector<Eigen::Vector2d>& opt_points,
                 const std::vector<Eigen::Vector2d>& left_boundary,
                 const std::vector<Eigen::Vector2d>& right_boundary,
                 const std::vector<double>& init_curv,
                 const std::vector<double>& opt_curv) const;

private:
    void optimizeTrajectory();
    void subscribeAndAdvertise();
    void initialize();

    ros::NodeHandle nh_;
    ros::Subscriber boundaries_sub_;

    struct Publishers {
        ros::Publisher optimized_path;
        ros::Publisher initial_curvature;
        ros::Publisher optimized_curvature;
        ros::Publisher left_boundary;
        ros::Publisher right_boundary;
    } pub_;

    struct Topics {
        std::string boundaries;
        std::string optimized_path;
        std::string initial_curvature;
        std::string optimized_curvature;
        std::string left_boundary;
        std::string right_boundary;
    } topics_;

    struct Frames {
        std::string robot;
        std::string world;
    } frames_;

    struct OptimizerParams {
        double weight;
        double last_point_shrink;
    } optimizer_params_;

    // Save boundaries time
    ros::Time boundaries_time_;

    // Internal data storage for boundaries and centerline splines
    std::shared_ptr<spline::BaseCubicSpline> centerline_spline_;
    std::shared_ptr<spline::BaseCubicSpline> left_boundary_spline_;
    std::shared_ptr<spline::BaseCubicSpline> right_boundary_spline_;
    std::shared_ptr<spline::BaseCubicSpline> optimized_trajectory_;

    // Solver pointer
    std::unique_ptr<spline::optimization::MinCurvatureOptimizer> optimizer_;
};

} // namespace min_curv_ros_wrapper
