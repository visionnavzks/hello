#include "motion_planner/shortcut_smoother.hpp"
#include <random>
#include <algorithm>

namespace motion_planner {

ShortcutSmoother::ShortcutSmoother(double step_size, double safety_margin, int max_iterations)
    : step_size_(step_size), safety_margin_(safety_margin), max_iterations_(max_iterations) {}

ShortcutSmoother::~ShortcutSmoother() {}

bool ShortcutSmoother::isPathValid(const Point2D& a, const Point2D& b, const ESDF& esdf) const {
    double dist = std::hypot(a.x - b.x, a.y - b.y);
    int steps = std::max(1, static_cast<int>(std::ceil(dist / step_size_)));
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        double x = a.x + t * (b.x - a.x);
        double y = a.y + t * (b.y - a.y);
        if (esdf.isInCollision(x, y, safety_margin_)) {
            return false;
        }
    }
    return true;
}

std::vector<Point2D> ShortcutSmoother::smooth(const std::vector<Point2D>& path, const ESDF& esdf) {
    if (path.size() <= 2) return path;

    std::vector<Point2D> smoothed = path;
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int iter = 0; iter < max_iterations_; ++iter) {
        if (smoothed.size() <= 2) break;

        std::uniform_int_distribution<> dist(0, smoothed.size() - 1);
        int i = dist(gen);
        int j = dist(gen);
        if (i > j) std::swap(i, j);
        if (j - i <= 1) continue;

        if (isPathValid(smoothed[i], smoothed[j], esdf)) {
            smoothed.erase(smoothed.begin() + i + 1, smoothed.begin() + j);
        }
    }

    return smoothed;
}

} // namespace motion_planner
