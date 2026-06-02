#ifndef MOTION_PLANNER_ASTAR_HPP
#define MOTION_PLANNER_ASTAR_HPP

#include "esdf.hpp"
#include <vector>
#include <memory>

namespace motion_planner {

struct AStarNode {
    Point2D point;
    double g_cost;
    double h_cost;
    double f_cost;
    std::shared_ptr<AStarNode> parent;

    AStarNode(Point2D p, double g, double h, std::shared_ptr<AStarNode> par = nullptr)
        : point(p), g_cost(g), h_cost(h), f_cost(g + h), parent(par) {}
};

class AStar2D {
public:
    AStar2D(double step_size, double safety_margin);
    ~AStar2D();

    std::vector<Point2D> plan(const Point2D& start, const Point2D& goal, const ESDF& esdf);

private:
    double step_size_;
    double safety_margin_;

    double heuristic(const Point2D& a, const Point2D& b) const;
    bool isPathValid(const Point2D& a, const Point2D& b, const ESDF& esdf) const;
    std::vector<Point2D> getNeighbors(const Point2D& current, const ESDF& esdf) const;
};

} // namespace motion_planner

#endif // MOTION_PLANNER_ASTAR_HPP
