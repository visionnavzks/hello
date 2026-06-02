#ifndef MOTION_PLANNER_REEDS_SHEPP_HPP
#define MOTION_PLANNER_REEDS_SHEPP_HPP

#include "common.hpp"
#include <vector>

namespace motion_planner {

struct ReedsSheppParams {
    double min_turn_radius = 1.0;
    double step_size = 0.1;
};

class ReedsSheppPlanner {
public:
    ReedsSheppPlanner(const ReedsSheppParams& params);
    ~ReedsSheppPlanner();

    std::vector<SE2Pose> plan(const SE2Pose& start, const SE2Pose& goal,
                              const std::vector<Point2D>& reference_path);

private:
    ReedsSheppParams params_;
    std::vector<SE2Pose> generateRSPath(const SE2Pose& start, const SE2Pose& goal);
};

} // namespace motion_planner

#endif // MOTION_PLANNER_REEDS_SHEPP_HPP
