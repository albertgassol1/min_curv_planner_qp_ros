#pragma once
#include <vector>
#include <stdexcept>
#include <Eigen/Dense>

namespace spline {
    class BaseCubicSpline {

    public:
        BaseCubicSpline();
        BaseCubicSpline(const std::vector<Eigen::Vector2d>& control_points);
        virtual const Eigen::Vector2d evaluateSpline(const double u, const std::size_t derivative_order) const = 0;
        virtual const double computeCurvature(const double u) const = 0;
        const size_t size() const;
        const size_t& degree() const;
        void setControlPoints(const std::vector<Eigen::Vector2d>& control_points);
        const std::vector<Eigen::Vector2d>& getControlPoints() const;

        virtual const std::pair<Eigen::MatrixXd, Eigen::MatrixXd> getCoefficients() const = 0;

    protected:
        std::vector<Eigen::Vector2d> control_points_;
        std::size_t degree_;
        virtual void initialize() = 0;
    };
} // namespace spline

