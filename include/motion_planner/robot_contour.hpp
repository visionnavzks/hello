#ifndef MOTION_PLANNER_ROBOT_CONTOUR_HPP
#define MOTION_PLANNER_ROBOT_CONTOUR_HPP

#include "common.hpp"
#include <vector>

namespace motion_planner {

enum class RobotType {
    CIRCULAR,
    RECTANGULAR
};

struct RobotConfig {
    RobotType type;
    double radius; // for circular
    double width;  // for rectangular
    double height; // for rectangular
};

class RobotContour {
public:
    RobotContour(const RobotConfig& config);
    ~RobotContour();

    std::vector<Point2D> getContourPoints(const SE2Pose& pose) const;
    double getBoundingRadius() const;

private:
    RobotConfig config_;
};

} // namespace motion_planner

#endif // MOTION_PLANNER_ROBOT_CONTOUR_HPP
