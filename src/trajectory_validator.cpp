#include "motion_planner/trajectory_validator.hpp"

namespace motion_planner {

TrajectoryValidator::TrajectoryValidator(const RobotContour& robot, double step_size)
    : robot_(robot), step_size_(step_size) {}
TrajectoryValidator::~TrajectoryValidator() {}

bool TrajectoryValidator::validateXYPath(const std::vector<Point2D>& path, const ESDF& esdf) {
    double radius = robot_.getBoundingRadius();
    for (size_t i = 0; i < path.size(); ++i) {
        if (esdf.isInCollision(path[i].x, path[i].y, radius)) {
            return false;
        }
    }
    return true;
}

bool TrajectoryValidator::validateSE2Trajectory(const std::vector<SE2Pose>& trajectory, const ESDF& esdf) {
    for (const auto& pose : trajectory) {
        auto contour = robot_.getContourPoints(pose);
        for (const auto& p : contour) {
            if (esdf.getDistance(p.x, p.y) < 0.0) {
                return false;
            }
        }
    }
    return true;
}

} // namespace motion_planner
