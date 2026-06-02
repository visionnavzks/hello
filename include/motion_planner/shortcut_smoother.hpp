#ifndef MOTION_PLANNER_SHORTCUT_SMOOTHER_HPP
#define MOTION_PLANNER_SHORTCUT_SMOOTHER_HPP

#include "esdf.hpp"
#include <vector>

namespace motion_planner {

class ShortcutSmoother {
public:
    ShortcutSmoother(double step_size, double safety_margin, int max_iterations = 100);
    ~ShortcutSmoother();

    std::vector<Point2D> smooth(const std::vector<Point2D>& path, const ESDF& esdf);

private:
    double step_size_;
    double safety_margin_;
    int max_iterations_;

    bool isPathValid(const Point2D& a, const Point2D& b, const ESDF& esdf) const;
};

} // namespace motion_planner

#endif // MOTION_PLANNER_SHORTCUT_SMOOTHER_HPP
