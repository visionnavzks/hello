#ifndef MOTION_PLANNER_COMMON_HPP
#define MOTION_PLANNER_COMMON_HPP

#include "esdf.hpp"

namespace motion_planner {

struct SE2Pose {
    double x, y, theta;
    SE2Pose() : x(0), y(0), theta(0) {}
    SE2Pose(double x, double y, double theta) : x(x), y(y), theta(theta) {}
};

} // namespace motion_planner

#endif // MOTION_PLANNER_COMMON_HPP
