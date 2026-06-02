#include "motion_planner/robot_contour.hpp"
#include <cmath>

namespace motion_planner {

RobotContour::RobotContour(const RobotConfig& config) : config_(config) {}
RobotContour::~RobotContour() {}

std::vector<Point2D> RobotContour::getContourPoints(const SE2Pose& pose) const {
    std::vector<Point2D> points;

    if (config_.type == RobotType::CIRCULAR) {
        int num_points = 16;
        for (int i = 0; i < num_points; ++i) {
            double angle = 2 * M_PI * i / num_points;
            Point2D p;
            p.x = pose.x + config_.radius * std::cos(pose.theta + angle);
            p.y = pose.y + config_.radius * std::sin(pose.theta + angle);
            points.push_back(p);
        }
    } else {
        double hw = config_.width / 2.0;
        double hh = config_.height / 2.0;
        std::vector<Point2D> local_points = {
            Point2D(-hw, -hh), Point2D(hw, -hh),
            Point2D(hw, hh), Point2D(-hw, hh)
        };
        for (const auto& local : local_points) {
            Point2D world;
            world.x = pose.x + local.x * std::cos(pose.theta) - local.y * std::sin(pose.theta);
            world.y = pose.y + local.x * std::sin(pose.theta) + local.y * std::cos(pose.theta);
            points.push_back(world);
        }
    }

    return points;
}

double RobotContour::getBoundingRadius() const {
    if (config_.type == RobotType::CIRCULAR) {
        return config_.radius;
    } else {
        return 0.5 * std::hypot(config_.width, config_.height);
    }
}

} // namespace motion_planner
