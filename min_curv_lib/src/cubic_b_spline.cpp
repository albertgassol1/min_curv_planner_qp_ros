#include "min_curv_lib/cubic_b_spline.hpp"

namespace spline{

CubicBSpline::CubicBSpline() : BaseCubicSpline() {}

CubicBSpline::CubicBSpline(const std::vector<Eigen::Vector2d>& control_points)
    : BaseCubicSpline(control_points) {
    initialize();
    }

void CubicBSpline::initialize(){
    const std::size_t numcontrol_points = control_points_.size();
    const std::size_t numKnots = numcontrol_points + degree_ + 1;
    knotVector_.reserve(numKnots);

    for (std::size_t i = 0; i <= degree_; ++i) {
        knotVector_[i] = 0.0;
    }

    for (std::size_t i = degree_ + 1; i < numKnots - degree_ - 1; ++i) {
        knotVector_[i] = (double)(i - degree_) / (numcontrol_points - degree_);
    }

    for (std::size_t i = numKnots - degree_ - 1; i < numKnots; ++i) {
        knotVector_[i] = 1.0;
    }
}

const std::size_t CubicBSpline::findKnotSpan(const std::size_t n, const double u) const{
    if (u == knotVector_[n+1])
        return n;
    
    std::size_t low = degree_;
    std::size_t high = n + 1;
    std::size_t mid = (low + high) / 2;
    while (u < knotVector_[mid] || u >= knotVector_[mid+1]) {
        if (u < knotVector_[mid])
            high = mid;
        else
            low = mid;
        mid = (low + high) / 2;
    }
    return mid;
}

// De Boor recursive function to evaluate basis functions
const double CubicBSpline::basisFunction(const std::size_t i, const std::size_t p, const double u) const{
    if (p == 0) {
        return (u >= knotVector_[i] && u < knotVector_[i+1]) ? 1.0 : 0.0;
    }

    double left = 0.0, right = 0.0;
    
    if (knotVector_[i + p] != knotVector_[i])
        left = (u - knotVector_[i]) / (knotVector_[i + p] - knotVector_[i]) * basisFunction(i, p - 1, u);
    
    if (knotVector_[i + p + 1] != knotVector_[i + 1])
        right = (knotVector_[i + p + 1] - u) / (knotVector_[i + p + 1] - knotVector_[i + 1]) * basisFunction(i + 1, p - 1, u);

    return left + right;
}

// Derivative of the basis function (1st and 2nd order)
const double CubicBSpline::basisFunctionDerivative(const std::size_t i, const std::size_t p, const double u, const std::size_t derivative_order) const{
    if (derivative_order == 0) {
        return basisFunction(i, p, u);
    }

    double left = 0.0, right = 0.0;

    if (knotVector_[i + p] != knotVector_[i]) {
        left = (p / (knotVector_[i + p] - knotVector_[i])) * basisFunctionDerivative(i, p - 1, u, derivative_order - 1);
    }

    if (knotVector_[i + p + 1] != knotVector_[i + 1]) {
        right = (p / (knotVector_[i + p + 1] - knotVector_[i + 1])) * basisFunctionDerivative(i + 1, p - 1, u, derivative_order - 1);
    }

    return left - right;
}

// Evaluate B-Spline or its derivatives at a given parameter u
const Eigen::Vector2d CubicBSpline::evaluateSpline(const double u, const std::size_t derivative_order) const {
    const std::size_t n = control_points_.size() - 1;
    const auto span = findKnotSpan(n, u);
    
    Eigen::Vector2d result(0.0, 0.0);
    
    for (std::size_t i = 0; i <= degree_; ++i) {
        double coeff = (derivative_order == 0) ? basisFunction(span - degree_ + i, degree_, u) :
                        basisFunctionDerivative(span - degree_ + i, degree_, u, derivative_order);
        result += coeff * control_points_[span - degree_ + i];
    }
    
    return result;
}

// Compute curvature from first and second derivatives
const double CubicBSpline::computeCurvature(const double u) const {
    const auto firstDerivative = evaluateSpline(u, 1);
    const auto secondDerivative = evaluateSpline(u, 2);
    double numerator = firstDerivative.x() * secondDerivative.y() - firstDerivative.y() * secondDerivative.x();
    double denominator = pow(firstDerivative.x() * firstDerivative.x() + firstDerivative.y() * firstDerivative.y(), 1.5);
    
    return fabs(numerator) / denominator;
}

const std::pair<Eigen::MatrixXd, Eigen::MatrixXd> CubicBSpline::getCoefficients() const {
    return std::make_pair(Eigen::MatrixXd::Zero(1, 1), Eigen::MatrixXd::Zero(1, 1));
}


}// namespace spline
