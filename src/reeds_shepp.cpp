#include "motion_planner/reeds_shepp.hpp"
#include <cmath>
#include <algorithm>

namespace motion_planner {

ReedsSheppPlanner::ReedsSheppPlanner(const ReedsSheppParams& params)
    : params_(params) {}

ReedsSheppPlanner::~ReedsSheppPlanner() {}

std::vector<SE2Pose> ReedsSheppPlanner::generateRSPath(const SE2Pose& start, const SE2Pose& goal) {
    std::vector<SE2Pose> path;
    path.push_back(start);

    double dx = goal.x - start.x;
    double dy = goal.y - start.y;
    double dist = std::hypot(dx, dy);
    int steps = std::max(1, static_cast<int>(dist / params_.step_size));

    for (int i = 1; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        SE2Pose pose;
        pose.x = start.x + t * dx;
        pose.y = start.y + t * dy;
        pose.theta = start.theta + t * (goal.theta - start.theta);
        path.push_back(pose);
    }

    return path;
}

std::vector<SE2Pose> ReedsSheppPlanner::plan(const SE2Pose& start, const SE2Pose& goal,
                                             const std::vector<Point2D>& reference_path) {
    std::vector<SE2Pose> path;

    if (reference_path.empty()) {
        return generateRSPath(start, goal);
    }

    path.push_back(start);
    for (size_t i = 1; i < reference_path.size(); ++i) {
        SE2Pose prev = path.back();
        SE2Pose next;
        next.x = reference_path[i].x;
        next.y = reference_path[i].y;
        double dx = reference_path[i].x - reference_path[i-1].x;
        double dy = reference_path[i].y - reference_path[i-1].y;
        next.theta = std::atan2(dy, dx);
        auto segment = generateRSPath(prev, next);
        path.insert(path.end(), segment.begin() + 1, segment.end());
    }
    auto final_segment = generateRSPath(path.back(), goal);
    path.insert(path.end(), final_segment.begin() + 1, final_segment.end());

    return path;
}

} // namespace motion_planner
