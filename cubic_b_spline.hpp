# pragma once
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include "base_cubic_spline.hpp"
namespace spline{
class CubicBSpline : public BaseCubicSpline{
    public:
        CubicBSpline(const std::vector<Eigen::Vector2d>& control_points);
        const Eigen::Vector2d evaluateSpline(const double u, const std::size_t derivative_order) const override;
        const double computeCurvature(const double u) const override;
        const std::pair<Eigen::MatrixXd, Eigen::MatrixXd> getCoefficients() const override;
    private:
        std::vector<double> knotVector_;
        void initialize();
        const std::size_t findKnotSpan(const std::size_t n, const double u) const;
        const double basisFunction(const std::size_t i, const std::size_t p, const double u) const;
        const double basisFunctionDerivative(const std::size_t i, const std::size_t p, const double u, const std::size_t derivative_order) const;
};
}// namespace spline
