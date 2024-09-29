#pragma once
#include <vector>
#include <Eigen/Dense>


namespace spline {
namespace optimization {

struct KDTreeAdapter {
    KDTreeAdapter(const std::vector<Eigen::Vector2d>& points) : pts(points) {}
    std::vector<Eigen::Vector2d> pts;

    inline std::size_t kdtree_get_point_count() const { return pts.size(); }
    inline double kdtree_distance(const double *p1, const std::size_t idx_p2, std::size_t) const {
        const double d0 = p1[0] - pts[idx_p2].x();
        const double d1 = p1[1] - pts[idx_p2].y();
        return d0 * d0 + d1 * d1;
    }
    inline double kdtree_get_pt(const std::size_t idx, int dim) const {
        return dim == 0 ? pts[idx].x() : pts[idx].y();
    }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};
    
} // namespace optimization
} // namespace spline;