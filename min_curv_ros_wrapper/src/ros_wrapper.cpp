#include "min_curv_ros_wrapper/ros_wrapper.hpp"

namespace min_curv_ros_wrapper {

RosWrapper::RosWrapper(ros::NodeHandle& nh) : nh_(nh) {
    initialize();
    subscribeAndAdvertise();
}

void RosWrapper::initialize() {
    // Topics
    nh_.param<std::string>("topics/boundaries", topics_.boundaries, "/initial/boundaries");
    nh_.param<std::string>("topics/optimized_path", topics_.optimized_path, "/optimized/centerline");
    nh_.param<std::string>("topics/initial_curvature", topics_.initial_curvature, "/initial/curvature");
    nh_.param<std::string>("topics/optimized_curvature", topics_.optimized_curvature, "/optimized/curvature");
    nh_.param<std::string>("topics/left_boundary", topics_.left_boundary, "/optimized/left_boundary");
    nh_.param<std::string>("topics/right_boundary", topics_.right_boundary, "/optimized/right_boundary");

    // Optimizer parameters
    int num_control_points, max_num_iterations;
    std::unique_ptr<spline::optimization::MinCurvatureParams> params = std::make_unique<spline::optimization::MinCurvatureParams>();
    nh_.param<bool>("optimizer/verbose", params->verbose, false);
    nh_.param<bool>("optimizer/constant_system_matrix", params->constant_system_matrix, false);
    nh_.param<int>("optimizer/num_control_points", num_control_points, 0);
    nh_.param<int>("optimizer/max_num_iterations", max_num_iterations, 100);
    nh_.param<bool>("optimizer/warm_start", params->warm_start, true);
    nh_.param<double>("optimizer/weight", optimizer_params_.weight, 0.5);
    nh_.param<double>("optimizer/last_point_shrink", optimizer_params_.last_point_shrink, 0.5);
    params->num_control_points = static_cast<std::size_t>(num_control_points);
    params->max_num_iterations = static_cast<std::size_t>(max_num_iterations);

    // Frames
    nh_.param<std::string>("frames/robot", frames_.robot, "base_link");
    nh_.param<std::string>("frames/world", frames_.world, "map");

    // Initialize the optimizer
    optimizer_ = std::make_unique<spline::optimization::MinCurvatureOptimizer>(std::move(params));

    // Initialize the splines
    centerline_spline_ = std::make_shared<spline::ParametricCubicSpline>();
    left_boundary_spline_ = std::make_shared<spline::ParametricCubicSpline>();
    right_boundary_spline_ = std::make_shared<spline::ParametricCubicSpline>();
    optimized_trajectory_ = std::make_shared<spline::ParametricCubicSpline>();

    optimizer_->setSplines(centerline_spline_, left_boundary_spline_, right_boundary_spline_);
}

void RosWrapper::subscribeAndAdvertise() {
    // Initialize the subscriber using the parameter
    boundaries_sub_ = nh_.subscribe(topics_.boundaries, 1, &RosWrapper::boundariesCallback, this);

    // Initialize publishers using the parameters
    pub_.optimized_path = nh_.advertise<nav_msgs::Path>(topics_.optimized_path, 1);
    pub_.initial_curvature = nh_.advertise<std_msgs::Float64>(topics_.initial_curvature, 1);
    pub_.optimized_curvature = nh_.advertise<std_msgs::Float64>(topics_.optimized_curvature, 1);
    pub_.left_boundary = nh_.advertise<nav_msgs::Path>(topics_.left_boundary, 1);
    pub_.right_boundary = nh_.advertise<nav_msgs::Path>(topics_.right_boundary, 1);
}

// Callback function to process the boundaries and centerline
void RosWrapper::boundariesCallback(const min_curv_msgs::Paths::ConstPtr& msg) {
    assert (msg->left_boundary.poses.size() == msg->right_boundary.poses.size() &&
            msg->left_boundary.poses.size() == msg->centerline.poses.size());

    boundaries_time_ = msg->header.stamp;
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
    optimizer_->setUp(optimizer_params_.last_point_shrink);
    // First optimization with a specific weight
    optimizer_->solve(optimized_trajectory_, optimizer_params_.weight);
    // Re-run the optimizer to smooth out the trajectory further
    optimizer_->setUp(optimizer_params_.last_point_shrink);
    optimizer_->solve(optimized_trajectory_, 1 - optimizer_params_.weight);
    optimized_trajectory_ = std::make_shared<spline::CubicBSpline>(optimized_trajectory_->getControlPoints());
    // Now we have the optimized trajectory, let's publish the result
    std::vector<Eigen::Vector2d> opt_points;
    std::vector<double> initial_curvatures;
    std::vector<double> optimized_curvatures;
    std::vector<Eigen::Vector2d> left_boundary;
    std::vector<Eigen::Vector2d> right_boundary;

    // Sample points from the optimized and initial splines
    for (double u = 0.0; u <= 1.0; u += 0.01) {
        opt_points.push_back(optimized_trajectory_->evaluateSpline(u, 0));
        initial_curvatures.push_back(centerline_spline_->computeCurvature(u));
        optimized_curvatures.push_back(optimized_trajectory_->computeCurvature(u));
        left_boundary.push_back(left_boundary_spline_->evaluateSpline(u, 0));
        right_boundary.push_back(right_boundary_spline_->evaluateSpline(u, 0));
    }

    // Publish the optimized path and curvature
    publish(opt_points, left_boundary, right_boundary, 
                         initial_curvatures, optimized_curvatures);
}

// Function to publish the optimized path and curvatures
void RosWrapper::publish(const std::vector<Eigen::Vector2d>& opt_points,
                         const std::vector<Eigen::Vector2d>& left_boundary,
                         const std::vector<Eigen::Vector2d>& right_boundary,
                         const std::vector<double>& init_curv,
                         const std::vector<double>& opt_curv) const {
    // Publish the optimized path
    nav_msgs::Path opt_path;
    opt_path.header.stamp = ros::Time::now();
    opt_path.header.frame_id = frames_.world;
    for (const auto& point : opt_points) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        opt_path.poses.push_back(pose);
    }
    pub_.optimized_path.publish(opt_path);

    // Publish boundaries
    nav_msgs::Path left_boundary_path;
    left_boundary_path.header.stamp = boundaries_time_;
    left_boundary_path.header.frame_id = frames_.world;
    for (const auto& point : left_boundary) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        left_boundary_path.poses.push_back(pose);
    }
    pub_.left_boundary.publish(left_boundary_path);

    nav_msgs::Path right_boundary_path;
    right_boundary_path.header.stamp = boundaries_time_;
    right_boundary_path.header.frame_id = frames_.world;
    for (const auto& point : right_boundary) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        right_boundary_path.poses.push_back(pose);
    }
    pub_.right_boundary.publish(right_boundary_path);

    // Publish the initial curvatures
    std_msgs::Float64MultiArray curv_init_msg;
    for (std::size_t i = 0; i < init_curv.size(); ++i) {
        curv_init_msg.data.push_back(init_curv[i]);
    }
    pub_.initial_curvature.publish(curv_init_msg);

    // Publish the optimized curvatures
    std_msgs::Float64MultiArray curv_opt_msg;
    for (std::size_t i = 0; i < opt_curv.size(); ++i) {
        curv_opt_msg.data.push_back(opt_curv[i]);
    }
    pub_.optimized_curvature.publish(curv_opt_msg);

    ROS_INFO("[min_curv_ros_wrapper] Optimized path and curvature have been published.");
}

} // namespace min_curv_ros_wrapper
