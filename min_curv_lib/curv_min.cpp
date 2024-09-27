#include "curv_min.hpp"

namespace spline {
namespace optimization {

MinCurvatureOptimizer::MinCurvatureOptimizer(std::unique_ptr<MinCurvatureParams> params) {
    // Initialize OSQP solver
    if (params) {
        params_ = std::move(params);
    } else {
        params_ = std::make_unique<MinCurvatureParams>();
    }
    solver_ = std::make_unique<OsqpEigen::Solver>();
    solver_->settings()->setVerbosity(params_->verbose);
    solver_->settings()->setMaxIteration(100); 
    solver_->settings()->setWarmStart(true);
    if (params_->constant_system_matrix) {
        setSystemMatrixInverse();
    }
}

void MinCurvatureOptimizer::setUp(const std::shared_ptr<BaseCubicSpline>& ref_spline,
                                  const std::shared_ptr<BaseCubicSpline>& left_spline,
                                  const std::shared_ptr<BaseCubicSpline>& right_spline,
                                  const double last_point_shrink) {
    ref_spline_ = ref_spline;
    left_spline_ = left_spline;
    right_spline_ = right_spline;
    
    setupQP(last_point_shrink);
}

void MinCurvatureOptimizer::setSystemMatrixInverse(const std::size_t size) {
    system_inverse_ = 
}

void MinCurvatureOptimizer::computeHessianAndLinear() {
    // Get normal vectors from coefficients 
    // Normal vector is the derivative of the spline, wich are coefficients b
    const std::size_t num_control_points = ref_spline_->size();
    const auto coefficients = ref_spline_->getCoefficients();
    normal_vectors_.resize(num_control_points, 2);
    normal_vectors_.col(0) = -coefficients.second.row(1);
    normal_vectors_.col(1) = coefficients.first.row(1);

    // Normalization of normal vectors
    normal_vectors_.rowwise().normalize();

    // Calculate A matrix (later updated in for loop)
    const std::size_t size_A = 4 * num_control_points;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(size_A, size_A);
    A.block<5, 7>(0, 0) <<  1.,  0.,  0.,  0.,  0.,  0.,  0.,
                            0.,  0.,  2.,  0.,  0.,  0.,  0.,
                            1.,  1.,  1.,  1.,  0.,  0.,  0.,
                            0.,  1.,  2.,  3.,  0., -1.,  0.,
                            0.,  0.,  1.,  3.,  0.,  0., -1.;
    A(size_A - 3, size_A - 4) = 1;
    A(size_A - 2, size_A - 2) = 2;
    A(size_A - 1, size_A - 1) = 1;

    // Compute P_xx, P_xy, P_yy
    Eigen::VectorXd square_normals = (normal_vectors_.col(0).array().square() + normal_vectors_.col(1).array().square());
    Eigen::MatrixXd P_xx = (normal_vectors_.col(0).array().square().array() / square_normals.array()).matrix().asDiagonal();
    Eigen::MatrixXd P_yy = (normal_vectors_.col(1).array().square().array() / square_normals.array()).matrix().asDiagonal();
    Eigen::MatrixXd P_xy = ((2 * normal_vectors_.col(1).array() * normal_vectors_.col(0).array()).array() / square_normals.array()).matrix().asDiagonal();

    // Compute q_x, q_y, M_x, M_y and extraction matrix A_ex
    Eigen::VectorXd q_x = Eigen::VectorXd::Zero(size_A);
    Eigen::VectorXd q_y = Eigen::VectorXd::Zero(size_A);
    Eigen::MatrixXd M_x = Eigen::MatrixXd::Zero(size_A, num_control_points);
    Eigen::MatrixXd M_y = Eigen::MatrixXd::Zero(size_A, num_control_points);
    Eigen::MatrixXd A_ex = Eigen::MatrixXd::Zero(num_control_points, size_A);

    const auto& control_points = ref_spline_->getControlPoints();
    q_x(0) = control_points[0].x();
    q_x(2) = control_points[1].x();
    q_y(0) = control_points[0].y();
    q_y(2) = control_points[1].y();
    M_x(0, 0) = normal_vectors_(0, 0);
    M_x(2, 1) = normal_vectors_(1, 0);
    M_y(0, 0) = normal_vectors_(0, 1);
    M_y(2, 1) = normal_vectors_(1, 1);
    A_ex(0, 2) = 1;

    for (std::size_t i = 1; i < num_control_points - 1; ++i) {
        q_x(4 * i + 1) = control_points[i].x();
        q_x(4 * i + 2) = control_points[i + 1].x();
        q_y(4 * i + 1) = control_points[i].y();
        q_y(4 * i + 2) = control_points[i + 1].y();
        M_x(4 * i + 1, i) = normal_vectors_(i, 0);
        M_x(4 * i + 2, i + 1) = normal_vectors_(i + 1, 0);
        M_y(4 * i + 1, i) = normal_vectors_(i, 1);
        M_y(4 * i + 2, i + 1) = normal_vectors_(i + 1, 1);
        A_ex(i, 4 * i + 2) = 1;
        A.block<4, 7>(4*i+1, 4*i) << 1.,  0.,  0.,  0.,  0.,  0.,  0.,
                                     1.,  1.,  1.,  1.,  0.,  0.,  0.,
                                     0.,  1.,  2.,  3.,  0., -1.,  0.,
                                     0.,  0.,  1.,  3.,  0.,  0., -1.;
    }
    q_x(size_A - 3) = control_points[num_control_points - 1].x();
    q_y(size_A - 3) = control_points[num_control_points - 1].y();
    M_x(size_A - 3, num_control_points - 1) = normal_vectors_(num_control_points - 1, 0);
    M_y(size_A - 3, num_control_points - 1) = normal_vectors_(num_control_points - 1, 1);
    A_ex(num_control_points - 1, size_A - 2) = 1;

    // Compute quadratic (hessian) and linear (gradient) terms
    Eigen::MatrixXd T_c = 2 * A_ex * A.inverse();
    Eigen::MatrixXd T_nx = T_c * M_x;
    Eigen::MatrixXd T_ny = T_c * M_y;
    Eigen::MatrixXd tmp = T_nx.adjoint() * P_xx * T_nx + T_ny.adjoint() * P_xy * T_nx + T_ny.adjoint() * P_yy * T_ny;
    c_ = 2 * T_nx.adjoint() * P_xx.adjoint() * T_c * q_x + T_ny.adjoint() * P_xy.adjoint() * T_c * q_x + 
         2 * T_ny.adjoint() * P_yy.adjoint() * T_c * q_y + T_nx.adjoint() * P_xy.adjoint() * T_c * q_y;
    H_ = (tmp.adjoint() + tmp) / 2;
}

// TODO: Implement a function that computes the distance using the normal vectors
const Eigen::MatrixXd MinCurvatureOptimizer::getBoundaryDistance() const {
    const std::size_t num_control_points = ref_spline_->size();
    Eigen::MatrixXd distance(num_control_points, 2);
    for (std::size_t i = 0; i < num_control_points; ++i) {
        const auto ref_point = ref_spline_->evaluateSpline(static_cast<double>(i) / (num_control_points - 1), 0);
        const auto left_point = left_spline_->evaluateSpline(static_cast<double>(i) / (num_control_points - 1), 0);
        const auto right_point = right_spline_->evaluateSpline(static_cast<double>(i) / (num_control_points - 1), 0);
        distance(i, 0) = (ref_point - left_point).norm();
        distance(i, 1) = (ref_point - right_point).norm();
    }
    return distance;
}

void MinCurvatureOptimizer::computeConstraints(const double last_point_shrink) {
    std::size_t num_control_points = ref_spline_->size();
    const auto distance = getBoundaryDistance();
    lower_bound_ = -distance.col(1);
    upper_bound_ = distance.col(0);
    A_ = Eigen::MatrixXd::Identity(ref_spline_->size(), ref_spline_->size());
    // Set the first control point to be fixed (i.e. no moving along the normal vector)
    lower_bound_(0) = 0.0;
    upper_bound_(0) = 0.0;
    // Set the last control point to have a smaller range
    lower_bound_(num_control_points - 1) = last_point_shrink * lower_bound_(num_control_points - 1);
    upper_bound_(num_control_points - 1) = last_point_shrink * upper_bound_(num_control_points - 1);
}

void MinCurvatureOptimizer::setupQP(const double last_point_shrink) {
    // Assert that last_point_shrink is in the range [0, 1]
    assert(last_point_shrink >= 0.0 && last_point_shrink <= 1.0);
    solver_->clearSolver();
    solver_->data()->clearHessianMatrix();
    solver_->data()->clearLinearConstraintsMatrix();
    computeHessianAndLinear();
    computeConstraints(last_point_shrink);

    // Lambda function to convert Eigen::MatrixXd to sparse matrix
    auto toSparse = [](const Eigen::MatrixXd& matrix) {
        Eigen::SparseMatrix<double> sparse(matrix.rows(), matrix.cols());
        for (int i = 0; i < matrix.outerSize(); ++i) {
            for (Eigen::MatrixXd::InnerIterator it(matrix, i); it; ++it) {
                sparse.insert(it.row(), it.col()) = it.value();
            }
        }
        sparse.makeCompressed();
        return sparse;
    };
    
    // Configure OSQP solver
    std::size_t num_control_points = ref_spline_->size();
    solver_->data()->setNumberOfVariables(num_control_points);
    solver_->data()->setNumberOfConstraints(num_control_points);
    solver_->data()->setHessianMatrix(toSparse(H_));
    solver_->data()->setGradient(c_);
    solver_->data()->setLinearConstraintsMatrix(toSparse(A_));
    solver_->data()->setLowerBound(lower_bound_);
    solver_->data()->setUpperBound(upper_bound_);
}

void MinCurvatureOptimizer::solve(std::shared_ptr<BaseCubicSpline>& opt_traj, const double normal_weight) {
    // Solve the QP problem
    solver_->initSolver();
    solver_->solveProblem();
    
    // Retrieve the solution (optimized control points)
    Eigen::VectorXd solution = normal_weight * solver_->getSolution();
    
    // Extract optimized control points (2D points for x and y)
    std::vector<Eigen::Vector2d> optimized_control_points(ref_spline_->size());
    const auto& control_points = ref_spline_->getControlPoints();
    for (std::size_t i = 0; i < ref_spline_->size(); ++i) {
        optimized_control_points[i].x() = control_points[i].x() + solution(i) * normal_vectors_(i, 0);
        optimized_control_points[i].y() = control_points[i].y() + solution(i) * normal_vectors_(i, 1);
    }
    opt_traj->setControlPoints(optimized_control_points);
}
 
} // namespace optimization
} // namespace spline