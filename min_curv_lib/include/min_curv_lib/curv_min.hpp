#pragma once

#include <OsqpEigen/OsqpEigen.h>
#include <vector>
#include <memory>
#include <Eigen/Dense>

#include "min_curv_lib/base_cubic_spline.hpp"

namespace spline {
namespace optimization {

struct MinCurvatureParams
{
    bool verbose = false;
    bool constant_system_matrix = false;
    std::size_t num_control_points = 0;
    std::size_t max_num_iterations = 100;
    bool warm_start = true;

    MinCurvatureParams() = default;
    MinCurvatureParams(bool verbose, 
                       bool constant_system_matrix, 
                       std::size_t num_control_points, 
                       std::size_t max_num_iterations, 
                       bool warm_start)
        : verbose(verbose), constant_system_matrix(constant_system_matrix), 
            num_control_points(num_control_points), max_num_iterations(max_num_iterations), 
            warm_start(warm_start) {}
};

class MinCurvatureOptimizer {
public:
    MinCurvatureOptimizer();
    MinCurvatureOptimizer(std::unique_ptr<MinCurvatureParams> params);
    void setSplines(const std::shared_ptr<BaseCubicSpline>& ref_spline,
                    const std::shared_ptr<BaseCubicSpline>& left_spline,
                    const std::shared_ptr<BaseCubicSpline>& right_spline);
    void setUp(const double last_point_shrink = 0.5);

    void solve(std::shared_ptr<BaseCubicSpline>& opt_traj, const double normal_weight = 1.0);

private:
    void initSolver();
    void setupQP(const double last_point_shrink);
    void computeHessianAndLinear();
    void computeConstraints(const double last_point_shrink);
    const Eigen::MatrixXd getBoundaryDistance() const; 
    void setSystemMatrixInverse(const std::size_t size);
    const Eigen::SparseMatrix<double> toSparseMatrix(const Eigen::MatrixXd& matrix) const;
    const Eigen::MatrixXd fromSparseMatrix(const Eigen::SparseMatrix<double>& sparse_matrix) const;
    
    // Data
    std::shared_ptr<BaseCubicSpline> ref_spline_ = nullptr;
    std::shared_ptr<BaseCubicSpline> left_spline_ = nullptr;
    std::shared_ptr<BaseCubicSpline> right_spline_ = nullptr;
    Eigen::MatrixXd normal_vectors_;

    // Parameters
    std::unique_ptr<MinCurvatureParams> params_;
    
    // OSQP Eigen objects
    std::unique_ptr<OsqpEigen::Solver> solver_;
    Eigen::MatrixXd H_;  // Quadratic hessin matrix
    Eigen::VectorXd c_;              // Linear cost vector
    Eigen::MatrixXd A_;  // Constraint matrix
    Eigen::MatrixXd system_inverse_;  // Inverse of the system matrix
    Eigen::VectorXd lower_bound_;     // Lower bound for constraints
    Eigen::VectorXd upper_bound_;     // Upper bound for constraints
};
} // namespace optimization
} // namespace spline
