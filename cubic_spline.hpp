#pragma once
#include <vector>
#include <stdexcept>
#include <Eigen/Dense>
#include "base_cubic_spline.hpp"

namespace spline{

class ParametricCubicSpline : public BaseCubicSpline{
public:
    ParametricCubicSpline(const std::vector<Eigen::Vector2d>& control_points);
    const Eigen::Vector2d evaluateSpline(const double u, const std::size_t derivative_order) const override;
    const double computeCurvature(const double u) const override;
    const std::pair<Eigen::MatrixXd, Eigen::MatrixXd> getCoefficients() const override;

private:
    std::vector<double> a_x_, b_x_, c_x_, d_x_; // Spline coefficients for x
    std::vector<double> a_y_, b_y_, c_y_, d_y_; // Spline coefficients for y

    // Helper function to compute the spline coefficients
    void initialize();

    // Helper function to find the correct interval and local u
    void getIntervalAndLocalT(const double u, std::size_t &i, double &local_u) const;
};
}// namespace spline