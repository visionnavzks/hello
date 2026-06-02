#ifndef MOTION_PLANNER_SE2_SMOOTHER_HPP
#define MOTION_PLANNER_SE2_SMOOTHER_HPP

#include "common.hpp"
#include "esdf.hpp"
#include <vector>

namespace motion_planner {

struct SE2SmootherParams {
    double smooth_weight = 0.5;
    double obstacle_weight = 0.5;
    double curvature_weight = 0.1;
    double jerk_weight = 0.01;
    double safety_margin = 0.5;
};

class SE2Smoother {
public:
    SE2Smoother(const SE2SmootherParams& params);
    ~SE2Smoother();

    std::vector<SE2Pose> smooth(const std::vector<SE2Pose>& path, const ESDF& esdf);

private:
    SE2SmootherParams params_;
};

} // namespace motion_planner

#endif // MOTION_PLANNER_SE2_SMOOTHER_HPP
