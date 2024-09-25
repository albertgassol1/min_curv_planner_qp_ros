#pragma once

#include "base_cubic_spline.hpp"
#include <OsqpEigen/OsqpEigen.h>
#include <vector>
#include <memory>
#include <Eigen/Dense>

namespace spline {
namespace optimization {

class MinCurvatureOptimizer {
public:
    MinCurvatureOptimizer(const bool verbose = false);
    void setUp(const std::shared_ptr<BaseCubicSpline>& ref_spline,
               const std::shared_ptr<BaseCubicSpline>& left_spline,
               const std::shared_ptr<BaseCubicSpline>& right_spline,
               const double last_point_shrink = 0.5);

    void solve(std::shared_ptr<BaseCubicSpline>& opt_traj, const double normal_weight = 1.0
    
    
    );

private:
    void setupQP(const double last_point_shrink);
    void computeHessianAndLinear();
    void computeConstraints(const double last_point_shrink);
    const Eigen::MatrixXd getBoundaryDistance() const; 
    
    // Data
    std::shared_ptr<BaseCubicSpline> ref_spline_ = nullptr;
    std::shared_ptr<BaseCubicSpline> left_spline_ = nullptr;
    std::shared_ptr<BaseCubicSpline> right_spline_ = nullptr;
    Eigen::MatrixXd normal_vectors_;
    
    // OSQP Eigen objects
    std::unique_ptr<OsqpEigen::Solver> solver_;
    Eigen::MatrixXd H_;  // Quadratic hessin matrix
    Eigen::VectorXd c_;              // Linear cost vector
    Eigen::MatrixXd A_;  // Constraint matrix
    Eigen::VectorXd lower_bound_;     // Lower bound for constraints
    Eigen::VectorXd upper_bound_;     // Upper bound for constraints
};
} // namespace optimization
} // namespace spline
