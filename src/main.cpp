#include "motion_planner/esdf.hpp"
#include "motion_planner/astar.hpp"
#include "motion_planner/shortcut_smoother.hpp"
#include "motion_planner/esdf_grad_smoother.hpp"
#include "motion_planner/reeds_shepp.hpp"
#include "motion_planner/se2_smoother.hpp"
#include "motion_planner/robot_contour.hpp"
#include "motion_planner/trajectory_validator.hpp"
#include <iostream>

using namespace motion_planner;

int main() {
    std::cout << "Motion Planner Demo" << std::endl;

    ESDF esdf(0.1, -10, 10, -10, 10);

    PolygonObstacle obstacle;
    obstacle.vertices = {Point2D(-2, -2), Point2D(2, -2), Point2D(2, 2), Point2D(-2, 2)};
    esdf.update({obstacle});

    Point2D start(-8, -8);
    Point2D goal(8, 8);

    AStar2D astar(0.5, 0.5);
    auto path = astar.plan(start, goal, esdf);
    std::cout << "A* path length: " << path.size() << std::endl;

    if (path.empty()) {
        std::cout << "No path found!" << std::endl;
        return 1;
    }

    ShortcutSmoother shortcut(0.1, 0.5, 100);
    auto shortcut_path = shortcut.smooth(path, esdf);
    std::cout << "Shortcut path length: " << shortcut_path.size() << std::endl;

    ESDFGradSmootherParams esdf_params;
    ESDFGradSmoother esdf_smoother(esdf_params);
    auto smooth_xy = esdf_smoother.smooth(shortcut_path, esdf);

    SE2Pose start_pose(-8, -8, M_PI/4);
    SE2Pose goal_pose(8, 8, M_PI/4);
    ReedsSheppParams rs_params;
    ReedsSheppPlanner rs_planner(rs_params);
    auto rs_trajectory = rs_planner.plan(start_pose, goal_pose, smooth_xy);

    SE2SmootherParams se2_params;
    SE2Smoother se2_smoother(se2_params);
    auto final_trajectory = se2_smoother.smooth(rs_trajectory, esdf);

    RobotConfig robot_config;
    robot_config.type = RobotType::RECTANGULAR;
    robot_config.width = 0.8;
    robot_config.height = 0.6;
    RobotContour robot(robot_config);

    TrajectoryValidator validator(robot, 0.1);
    bool valid = validator.validateSE2Trajectory(final_trajectory, esdf);
    std::cout << "Trajectory valid: " << (valid ? "Yes" : "No") << std::endl;

    return 0;
}
