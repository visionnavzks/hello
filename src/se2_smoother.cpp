#include "motion_planner/se2_smoother.hpp"
#include <ceres/ceres.h>
#include <cmath>

namespace motion_planner {

struct SE2State {
    double x, y, theta;
};

class SmoothnessCost {
public:
    SmoothnessCost(double weight) : weight_(weight) {}

    template <typename T>
    bool operator()(const T* const x_prev, const T* const y_prev, const T* const theta_prev,
                    const T* const x_curr, const T* const y_curr, const T* const theta_curr,
                    const T* const x_next, const T* const y_next, const T* const theta_next,
                    T* residual) const {
        residual[0] = weight_ * (x_prev[0] - 2.0 * x_curr[0] + x_next[0]);
        residual[1] = weight_ * (y_prev[0] - 2.0 * y_curr[0] + y_next[0]);
        T delta_theta_prev = theta_curr[0] - theta_prev[0];
        T delta_theta_next = theta_next[0] - theta_curr[0];
        delta_theta_prev = atan2(sin(delta_theta_prev), cos(delta_theta_prev));
        delta_theta_next = atan2(sin(delta_theta_next), cos(delta_theta_next));
        residual[2] = weight_ * (delta_theta_prev - delta_theta_next);
        return true;
    }

private:
    double weight_;
};

class ObstacleCost {
public:
    ObstacleCost(const ESDF& esdf, double weight, double safety_margin)
        : esdf_(esdf), weight_(weight), safety_margin_(safety_margin) {}

    template <typename T>
    bool operator()(const T* const x, const T* const y, T* residual) const {
        T dist = T(esdf_.getDistance(static_cast<double>(x[0]), static_cast<double>(y[0])));
        if (dist < T(safety_margin_)) {
            residual[0] = weight_ * (T(safety_margin_) - dist);
        } else {
            residual[0] = T(0.0);
        }
        return true;
    }

private:
    const ESDF& esdf_;
    double weight_;
    double safety_margin_;
};

SE2Smoother::SE2Smoother(const SE2SmootherParams& params) : params_(params) {}
SE2Smoother::~SE2Smoother() {}

std::vector<SE2Pose> SE2Smoother::smooth(const std::vector<SE2Pose>& path, const ESDF& esdf) {
    if (path.size() <= 2) return path;

    std::vector<double> x, y, theta;
    for (const auto& pose : path) {
        x.push_back(pose.x);
        y.push_back(pose.y);
        theta.push_back(pose.theta);
    }

    ceres::Problem problem;

    for (size_t i = 1; i < path.size() - 1; ++i) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<SmoothnessCost, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1>(
                new SmoothnessCost(params_.smooth_weight)),
            nullptr,
            &x[i-1], &y[i-1], &theta[i-1],
            &x[i], &y[i], &theta[i],
            &x[i+1], &y[i+1], &theta[i+1]);

        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<ObstacleCost, 1, 1, 1>(
                new ObstacleCost(esdf, params_.obstacle_weight, params_.safety_margin)),
            nullptr,
            &x[i], &y[i]);
    }

    problem.SetParameterBlockConstant(&x[0]);
    problem.SetParameterBlockConstant(&y[0]);
    problem.SetParameterBlockConstant(&theta[0]);
    problem.SetParameterBlockConstant(&x.back());
    problem.SetParameterBlockConstant(&y.back());
    problem.SetParameterBlockConstant(&theta.back());

    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::vector<SE2Pose> smoothed;
    for (size_t i = 0; i < path.size(); ++i) {
        smoothed.emplace_back(x[i], y[i], theta[i]);
    }

    return smoothed;
}

} // namespace motion_planner
