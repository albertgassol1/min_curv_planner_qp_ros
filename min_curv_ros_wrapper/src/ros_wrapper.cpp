#include "min_curv_ros_wrapper/ros_wrapper.h"

namespace min_curv_ros_wrapper {

// Constructor to initialize subscribers, publishers, and the optimizer
RosWrapper::RosWrapper(ros::NodeHandle& nh) : nh_(nh) {
    // Initialize the single subscriber to boundaries (left, right, and centerline combined)
    boundaries_sub_ = nh_.subscribe("boundaries", 1, &RosWrapper::boundariesCallback, this);
    
    // Initialize publishers for the optimized path and curvatures
    pub_.optimized_path_pub_ = nh_.advertise<nav_msgs::Path>("optimized_path", 1);
    pub_.initial_curvature_pub_ = nh_.advertise<std_msgs::Float64>("initial_curvature", 1);
    pub_.optimized_curvature_pub_ = nh_.advertise<std_msgs::Float64>("optimized_curvature", 1);
    
    // Initialize the optimizer
    optimizer_ = std::make_unique<spline::optimization::MinCurvatureOptimizer>(false); // 'false' for non-verbose mode

    // Initialize the splines
    centerline_spline_ = std::make_shared<spline::ParametricCubicSpline>();
    left_boundary_spline_ = std::make_shared<spline::ParametricCubicSpline>();
    right_boundary_spline_ = std::make_shared<spline::ParametricCubicSpline>();
    optimized_trajectory_ = std::make_shared<spline::ParametricCubicSpline>();
}

// Callback function to process the boundaries and centerline
void RosWrapper::boundariesCallback(const min_curv_ros_wrapper::Paths::ConstPtr& msg) {
    // Extract the boundaries and centerline points from the message
    std::vector<Eigen::Vector2d> left_boundary;
    std::vector<Eigen::Vector2d> right_boundary;
    std::vector<Eigen::Vector2d> centerline;

    for (const auto& point : msg->left_boundary.poses) {
        left_boundary.emplace_back(point.pose.position.x, point.pose.position.y);
    }
    for (const auto& point : msg->right_boundary.poses) {
        right_boundary.emplace_back(point.pose.position.x, point.pose.position.y);
    }
    for (const auto& point : msg->centerline.poses) {
        centerline.emplace_back(point.pose.position.x, point.pose.position.y);
    }

    // Set the splines for left, right, and centerline
    left_boundary_spline_->setControlPoints(left_boundary);
    right_boundary_spline_->setControlPoints(right_boundary);
    centerline_spline_->setControlPoints(centerline);

    // Call the trajectory optimization function
    optimizeTrajectory();
}

// Function to optimize the trajectory using the minimum curvature optimization
void RosWrapper::optimizeTrajectory() {
    if (!left_boundary_spline_ || !right_boundary_spline_ || !centerline_spline_) {
        if (left_boundary_spline_->size() == 0 || right_boundary_spline_->size() == 0 || centerline_spline_->size() == 0) {
            ROS_WARN("Cannot optimize trajectory: Splines are not initialized properly.");
        }
        return;
    }
    // Set up the optimizer with the centerline, left, and right boundaries
    optimizer_->setUp(centerline_spline_, left_boundary_spline_, right_boundary_spline_, 0.5);

    // First optimization with a specific weight
    optimizer_->solve(optimized_trajectory_, 0.5);

    // Re-run the optimizer to smooth out the trajectory further
    optimizer_->setUp(optimized_trajectory_, left_boundary_spline_, right_boundary_spline_, 0.5);
    optimizer_->solve(optimized_trajectory_, 0.5);
    optimized_trajectory_ = std::shared_ptr<spline::CubicBSpline>(optimized_trajectory_->getControlPoints());

    // Now we have the optimized trajectory, let's publish the result
    std::vector<Eigen::Vector2d> opt_points;
    std::vector<double> initial_curvatures;
    std::vector<double> optimized_curvatures;

    // Sample points from the optimized and initial splines
    for (double u = 0.0; u <= 1.0; u += 0.01) {
        opt_points.push_back(optimized_trajectory_->evaluateSpline(u, 0));
        initial_curvatures.push_back(centerline_spline_->computeCurvature(u));
        optimized_curvatures.push_back(optimized_trajectory_->computeCurvature(u));
    }

    // Publish the optimized path and curvature
    publishOptimizedPath(opt_points, initial_curvatures, optimized_curvatures);
}

// Function to publish the optimized path and curvatures
void RosWrapper::publishOptimizedPath(const std::vector<Eigen::Vector2d>& opt_points,
                                      const std::vector<double>& init_curv,
                                      const std::vector<double>& opt_curv) const {
    // Publish the optimized path
    nav_msgs::Path opt_path;
    opt_path.header.stamp = ros::Time::now();
    opt_path.header.frame_id = "map";

    for (const auto& point : opt_points) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        opt_path.poses.push_back(pose);
    }
    pub_.optimized_path_pub_.publish(opt_path);

    // Publish the initial curvatures
    for (const auto& curv : init_curv) {
        std_msgs::Float64 curv_msg;
        curv_msg.data = curv;
        pub_.initial_curvature_pub_.publish(curv_msg);
    }

    // Publish the optimized curvatures
    for (const auto& curv : opt_curv) {
        std_msgs::Float64 curv_msg;
        curv_msg.data = curv;
        pub_.optimized_curvature_pub_.publish(curv_msg);
    }

    ROS_INFO("Optimized path and curvature have been published.");
}

} // namespace min_curv_ros_wrapper
