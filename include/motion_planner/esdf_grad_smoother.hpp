#ifndef MOTION_PLANNER_ESDF_GRAD_SMOOTHER_HPP
#define MOTION_PLANNER_ESDF_GRAD_SMOOTHER_HPP

#include "esdf.hpp"
#include <vector>

namespace motion_planner {

struct ESDFGradSmootherParams {
    double smooth_weight = 0.5;
    double obstacle_weight = 0.5;
    double curvature_weight = 0.1;
    double learning_rate = 0.01;
    int max_iterations = 1000;
    double convergence_tol = 1e-4;
};

class ESDFGradSmoother {
public:
    ESDFGradSmoother(const ESDFGradSmootherParams& params);
    ~ESDFGradSmoother();

    std::vector<Point2D> smooth(const std::vector<Point2D>& path, const ESDF& esdf);

private:
    ESDFGradSmootherParams params_;
};

} // namespace motion_planner

#endif // MOTION_PLANNER_ESDF_GRAD_SMOOTHER_HPP
