#include "motion_planner/esdf_grad_smoother.hpp"
#include <cmath>

namespace motion_planner {

ESDFGradSmoother::ESDFGradSmoother(const ESDFGradSmootherParams& params)
    : params_(params) {}

ESDFGradSmoother::~ESDFGradSmoother() {}

std::vector<Point2D> ESDFGradSmoother::smooth(const std::vector<Point2D>& path, const ESDF& esdf) {
    if (path.size() <= 2) return path;

    std::vector<Point2D> smoothed = path;
    int n = smoothed.size();

    for (int iter = 0; iter < params_.max_iterations; ++iter) {
        double max_update = 0.0;

        for (int i = 1; i < n - 1; ++i) {
            Eigen::Vector2d grad_smooth(0, 0);
            Eigen::Vector2d grad_obstacle(0, 0);
            Eigen::Vector2d grad_curvature(0, 0);

            grad_smooth = (smoothed[i-1].x - 2*smoothed[i].x + smoothed[i+1].x) * Eigen::Vector2d::UnitX() +
                          (smoothed[i-1].y - 2*smoothed[i].y + smoothed[i+1].y) * Eigen::Vector2d::UnitY();
            grad_smooth *= params_.smooth_weight;

            double dist = esdf.getDistance(smoothed[i].x, smoothed[i].y);
            if (dist < 1.0) {
                Eigen::Vector2d esdf_grad = esdf.getGradient(smoothed[i].x, smoothed[i].y);
                grad_obstacle = -params_.obstacle_weight * (1.0 - dist) * esdf_grad;
            }

            if (i > 0 && i < n - 1) {
                Eigen::Vector2d a(smoothed[i].x - smoothed[i-1].x, smoothed[i].y - smoothed[i-1].y);
                Eigen::Vector2d b(smoothed[i+1].x - smoothed[i].x, smoothed[i+1].y - smoothed[i].y);
                double curvature = (a.x()*b.y() - a.y()*b.x()) / std::pow(std::hypot(a.x(), a.y()), 3);
                grad_curvature = params_.curvature_weight * curvature * Eigen::Vector2d(a.y(), -a.x()).normalized();
            }

            Eigen::Vector2d total_grad = grad_smooth + grad_obstacle + grad_curvature;
            double update_x = params_.learning_rate * total_grad.x();
            double update_y = params_.learning_rate * total_grad.y();

            smoothed[i].x += update_x;
            smoothed[i].y += update_y;

            max_update = std::max(max_update, std::hypot(update_x, update_y));
        }

        if (max_update < params_.convergence_tol) {
            break;
        }
    }

    return smoothed;
}

} // namespace motion_planner
