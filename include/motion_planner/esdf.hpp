#ifndef MOTION_PLANNER_ESDF_HPP
#define MOTION_PLANNER_ESDF_HPP

#include <vector>
#include <cmath>
#include <utility>
#include <Eigen/Dense>

namespace motion_planner {

struct Point2D {
    double x, y;
    Point2D() : x(0), y(0) {}
    Point2D(double x, double y) : x(x), y(y) {}

    bool operator==(const Point2D& other) const {
        return std::abs(x - other.x) < 1e-6 && std::abs(y - other.y) < 1e-6;
    }
};

struct PolygonObstacle {
    std::vector<Point2D> vertices;
};

} // namespace motion_planner

namespace std {
    template <> struct hash<motion_planner::Point2D> {
        size_t operator()(const motion_planner::Point2D& p) const {
            size_t h1 = hash<double>()(p.x);
            size_t h2 = hash<double>()(p.y);
            return h1 ^ (h2 << 1);
        }
    };
}

namespace motion_planner {

class ESDF {
public:
    ESDF(double resolution, double x_min, double x_max, double y_min, double y_max);
    ~ESDF();

    void update(const std::vector<PolygonObstacle>& obstacles);
    double getDistance(double x, double y) const;
    Eigen::Vector2d getGradient(double x, double y) const;
    bool isInCollision(double x, double y, double radius) const;

private:
    double resolution_;
    double x_min_, x_max_, y_min_, y_max_;
    int width_, height_;
    std::vector<std::vector<double>> distance_field_;

    Point2D worldToGrid(double x, double y) const;
    Point2D gridToWorld(int i, int j) const;
    void computeESDF(const std::vector<PolygonObstacle>& obstacles);
    double pointToSegmentDistance(const Point2D& p, const Point2D& a, const Point2D& b) const;
};

} // namespace motion_planner

#endif // MOTION_PLANNER_ESDF_HPP
