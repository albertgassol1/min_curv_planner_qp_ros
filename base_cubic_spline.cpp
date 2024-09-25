#include "base_cubic_spline.hpp"

namespace spline
{

BaseCubicSpline::BaseCubicSpline(const std::vector<Eigen::Vector2d>& control_points)
    : control_points_(control_points), degree_(3){} 

void BaseCubicSpline::setControlPoints(const std::vector<Eigen::Vector2d>& control_points){
    control_points_ = control_points;
    initialize();
}

const std::size_t BaseCubicSpline::size() const{
    return control_points_.size();
}

const std::size_t& BaseCubicSpline::degree() const{
    return degree_;
}   

const std::vector<Eigen::Vector2d>& BaseCubicSpline::getControlPoints() const{
    return control_points_;
}
}// namespace spline
