#include "motion_planner/astar.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

namespace motion_planner {

AStar2D::AStar2D(double step_size, double safety_margin)
    : step_size_(step_size), safety_margin_(safety_margin) {}

AStar2D::~AStar2D() {}

double AStar2D::heuristic(const Point2D& a, const Point2D& b) const {
    return std::hypot(a.x - b.x, a.y - b.y);
}

bool AStar2D::isPathValid(const Point2D& a, const Point2D& b, const ESDF& esdf) const {
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

std::vector<Point2D> AStar2D::getNeighbors(const Point2D& current, const ESDF& esdf) const {
    std::vector<Point2D> neighbors;
    const double angles[] = {0, M_PI / 4, M_PI / 2, 3 * M_PI / 4,
                             M_PI, 5 * M_PI / 4, 3 * M_PI / 2, 7 * M_PI / 4};
    for (double angle : angles) {
        Point2D next(current.x + step_size_ * std::cos(angle),
                     current.y + step_size_ * std::sin(angle));
        if (isPathValid(current, next, esdf)) {
            neighbors.push_back(next);
        }
    }
    return neighbors;
}

std::vector<Point2D> AStar2D::plan(const Point2D& start, const Point2D& goal, const ESDF& esdf) {
    auto compare = [](const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) {
        return a->f_cost > b->f_cost;
    };
    std::priority_queue<std::shared_ptr<AStarNode>,
                        std::vector<std::shared_ptr<AStarNode>>,
                        decltype(compare)> open_set(compare);

    auto hash_point = [](const Point2D& p) {
        return std::hash<double>()(p.x) ^ (std::hash<double>()(p.y) << 1);
    };
    std::unordered_map<Point2D, double, decltype(hash_point)> g_score(10, hash_point);

    auto start_node = std::make_shared<AStarNode>(start, 0.0, heuristic(start, goal));
    open_set.push(start_node);
    g_score[start] = 0.0;

    while (!open_set.empty()) {
        auto current = open_set.top();
        open_set.pop();

        if (heuristic(current->point, goal) < step_size_) {
            std::vector<Point2D> path;
            auto node = current;
            while (node) {
                path.push_back(node->point);
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
            path.push_back(goal);
            return path;
        }

        for (const auto& neighbor : getNeighbors(current->point, esdf)) {
            double tentative_g = current->g_cost + heuristic(current->point, neighbor);
            auto it = g_score.find(neighbor);
            if (it == g_score.end() || tentative_g < it->second) {
                g_score[neighbor] = tentative_g;
                auto neighbor_node = std::make_shared<AStarNode>(
                    neighbor, tentative_g, heuristic(neighbor, goal), current);
                open_set.push(neighbor_node);
            }
        }
    }

    return {};
}

} // namespace motion_planner
