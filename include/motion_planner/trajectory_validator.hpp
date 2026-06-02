#ifndef MOTION_PLANNER_TRAJECTORY_VALIDATOR_HPP
#define MOTION_PLANNER_TRAJECTORY_VALIDATOR_HPP

#include "common.hpp"
#include "esdf.hpp"
#include "robot_contour.hpp"
#include <vector>

namespace motion_planner {

class TrajectoryValidator {
public:
    TrajectoryValidator(const RobotContour& robot, double step_size);
    ~TrajectoryValidator();

    bool validateXYPath(const std::vector<Point2D>& path, const ESDF& esdf);
    bool validateSE2Trajectory(const std::vector<SE2Pose>& trajectory, const ESDF& esdf);

private:
    const RobotContour& robot_;
    double step_size_;
};

} // namespace motion_planner

#endif // MOTION_PLANNER_TRAJECTORY_VALIDATOR_HPP
