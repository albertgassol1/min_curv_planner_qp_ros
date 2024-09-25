#include "cubic_spline.hpp"

namespace spline {

    ParametricCubicSpline::ParametricCubicSpline(const std::vector<Eigen::Vector2d>& control_points)
        : BaseCubicSpline(control_points) {
        initialize();
    }

    // Evaluate the parametric spline at t (0 <= t <= 1)
    const Eigen::Vector2d ParametricCubicSpline::evaluateSpline(const double u, const std::size_t derivative_order) const {
        std::size_t i;
        double local_u;
        getIntervalAndLocalT(u, i, local_u);

        // Compute x and y based on the derivative order and spline coefficients
        // Use the computed spline coefficients for a_x, b_x, c_x, d_x and a_y, b_y, c_y, d_y
        double x_val, y_val;
        if (derivative_order == 0) {
            x_val = a_x_[i] + b_x_[i] * local_u + c_x_[i] * local_u * local_u + d_x_[i] * local_u * local_u * local_u;
            y_val = a_y_[i] + b_y_[i] * local_u + c_y_[i] * local_u * local_u + d_y_[i] * local_u * local_u * local_u;
        } else if (derivative_order == 1) {
            x_val = b_x_[i] + 2 * c_x_[i] * local_u + 3 * d_x_[i] * local_u * local_u;
            y_val = b_y_[i] + 2 * c_y_[i] * local_u + 3 * d_y_[i] * local_u * local_u;
        } else if (derivative_order == 2) {
            x_val = 2 * c_x_[i] + 6 * d_x_[i] * local_u;
            y_val = 2 * c_y_[i] + 6 * d_y_[i] * local_u;
        } else {
            throw std::invalid_argument("Unsupported derivative order.");
        }

        return Eigen::Vector2d(x_val, y_val);
    }

    // Compute curvature at the parameter u
    const double ParametricCubicSpline::computeCurvature(const double u) const {
        const auto firstDerivative = evaluateSpline(u, 1);
        const auto secondDerivative = evaluateSpline(u, 2);
        double numerator = firstDerivative.x() * secondDerivative.y() - firstDerivative.y() * secondDerivative.x();
        double denominator = std::pow(firstDerivative.x() * firstDerivative.x() + firstDerivative.y() * firstDerivative.y(), 1.5);

        return fabs(numerator) / denominator;
    }

    // Helper function to compute the spline coefficients

    void ParametricCubicSpline::initialize() {
        const std::size_t num_control_points = control_points_.size();
        Eigen::VectorXd h = Eigen::VectorXd::Ones(num_control_points - 1);
        std::vector<Eigen::Vector2d> alpha(num_control_points - 1), z(num_control_points);
        std::vector<double> l(num_control_points), mu(num_control_points);

        // Step 2: Compute the alpha values
        for (std::size_t i = 1; i < num_control_points - 1; ++i) {
            alpha[i] = (3.0 / h[i] * (control_points_[i + 1] - control_points_[i])) - (3.0 / h[i - 1] * (control_points_[i] - control_points_[i - 1]));
        }

        // Step 3: Tridiagonal system setup and solution (Thomas algorithm)
        l[0] = 1.0; mu[0] = 0.0; z[0] = Eigen::Vector2d::Zero();
        for (std::size_t i = 1; i < num_control_points - 1; ++i) {
            l[i] = 2.0 * (h[i - 1] + h[i]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }
        l[num_control_points - 1] = 1.0;
        z[num_control_points - 1] = Eigen::Vector2d::Zero();

        // Step 4: Compute the spline coefficients
        a_x_.resize(num_control_points);
        a_y_.resize(num_control_points);
        for (std::size_t i = 0; i < num_control_points; ++i) {
            a_x_[i] = control_points_[i].x();
            a_y_[i] = control_points_[i].y();
        }

        // Resize coefficients to n
        b_x_.resize(num_control_points);
        c_x_.resize(num_control_points);
        d_x_.resize(num_control_points);
        b_y_.resize(num_control_points);
        c_y_.resize(num_control_points);
        d_y_.resize(num_control_points);

        for (int j = num_control_points - 2; j >= 0; j--) {
            c_x_[j] = z[j].x() - mu[j] * c_x_[j + 1];
            b_x_[j] = (a_x_[j + 1] - a_x_[j]) / h[j] - h[j] * (c_x_[j + 1] + 2.0 * c_x_[j]) / 3.0;
            d_x_[j] = (c_x_[j + 1] - c_x_[j]) / (3.0 * h[j]);

            c_y_[j] = z[j].y() - mu[j] * c_y_[j + 1];
            b_y_[j] = (a_y_[j + 1] - a_y_[j]) / h[j] - h[j] * (c_y_[j + 1] + 2.0 * c_y_[j]) / 3.0;
            d_y_[j] = (c_y_[j + 1] - c_y_[j]) / (3.0 * h[j]);
        }

        // Handle the last element separately
        b_x_[num_control_points - 1] = b_x_[num_control_points - 2];
        c_x_[num_control_points - 1] = c_x_[num_control_points - 2];
        d_x_[num_control_points - 1] = 0.0; // No third derivative at the last point
        b_y_[num_control_points - 1] = b_y_[num_control_points - 2];
        c_y_[num_control_points - 1] = c_y_[num_control_points - 2];
        d_y_[num_control_points - 1] = 0.0; // No third derivative at the last point
    }

    // Helper function to find the correct interval and local t
    void ParametricCubicSpline::getIntervalAndLocalT(const double u, std::size_t &i, double &local_u) const {
        if (u < 0.0 || u > 1.0) {
            throw std::out_of_range("t must be in the range [0, 1].");
        }

        // Convert t from [0, 1] to the spline parameter in [0, n-1]
        const std::size_t n = control_points_.size();
        double scaled_u = u * static_cast<double>(n - 1);
        i = static_cast<std::size_t>(scaled_u);
        if (i >= n - 1) {
            i = n - 2;  // Ensure we don't exceed bounds
        }
        local_u = scaled_u - i;
    }

    const std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ParametricCubicSpline::getCoefficients() const {
        Eigen::MatrixXd coefficients_x(4, control_points_.size());
        Eigen::MatrixXd coefficients_y(4, control_points_.size());
        for (std::size_t i = 0; i < control_points_.size(); ++i) {
            coefficients_x(0, i) = a_x_[i];
            coefficients_x(1, i) = b_x_[i];
            coefficients_x(2, i) = c_x_[i];
            coefficients_x(3, i) = d_x_[i];
            coefficients_y(0, i) = a_y_[i];
            coefficients_y(1, i) = b_y_[i];
            coefficients_y(2, i) = c_y_[i];
            coefficients_y(3, i) = d_y_[i];
        }
        return std::make_pair(coefficients_x, coefficients_y);
    }

} // namespace spline