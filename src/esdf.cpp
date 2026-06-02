#include "motion_planner/esdf.hpp"
#include <algorithm>
#include <limits>
#include <queue>

namespace motion_planner {

ESDF::ESDF(double resolution, double x_min, double x_max, double y_min, double y_max)
    : resolution_(resolution), x_min_(x_min), x_max_(x_max), y_min_(y_min), y_max_(y_max) {
    width_ = static_cast<int>(std::ceil((x_max_ - x_min_) / resolution_)) + 1;
    height_ = static_cast<int>(std::ceil((y_max_ - y_min_) / resolution_)) + 1;
    distance_field_.resize(height_, std::vector<double>(width_, std::numeric_limits<double>::infinity()));
}

ESDF::~ESDF() {}

Point2D ESDF::worldToGrid(double x, double y) const {
    int i = static_cast<int>(std::round((y - y_min_) / resolution_));
    int j = static_cast<int>(std::round((x - x_min_) / resolution_));
    return Point2D(static_cast<double>(i), static_cast<double>(j));
}

Point2D ESDF::gridToWorld(int i, int j) const {
    double x = x_min_ + j * resolution_;
    double y = y_min_ + i * resolution_;
    return Point2D(x, y);
}

double ESDF::pointToSegmentDistance(const Point2D& p, const Point2D& a, const Point2D& b) const {
    Eigen::Vector2d ap(p.x - a.x, p.y - a.y);
    Eigen::Vector2d ab(b.x - a.x, b.y - a.y);
    double ab_sq = ab.squaredNorm();
    if (ab_sq == 0) return ap.norm();
    double t = std::max(0.0, std::min(1.0, ap.dot(ab) / ab_sq));
    Eigen::Vector2d projection = a.x + t * ab.x() * Eigen::Vector2d::Ones() + 
                                  a.y + t * ab.y() * Eigen::Vector2d::Ones();
    projection.x() = a.x + t * ab.x();
    projection.y() = a.y + t * ab.y();
    return (Eigen::Vector2d(p.x, p.y) - projection).norm();
}

void ESDF::update(const std::vector<PolygonObstacle>& obstacles) {
    for (int i = 0; i < height_; ++i) {
        for (int j = 0; j < width_; ++j) {
            distance_field_[i][j] = std::numeric_limits<double>::infinity();
        }
    }

    computeESDF(obstacles);
}

void ESDF::computeESDF(const std::vector<PolygonObstacle>& obstacles) {
    std::vector<std::vector<bool>> occupied(height_, std::vector<bool>(width_, false));

    for (const auto& obstacle : obstacles) {
        for (int i = 0; i < height_; ++i) {
            for (int j = 0; j < width_; ++j) {
                Point2D p = gridToWorld(i, j);
                bool inside = false;
                int n = obstacle.vertices.size();
                for (int k = 0, l = n - 1; k < n; l = k++) {
                    const Point2D& a = obstacle.vertices[k];
                    const Point2D& b = obstacle.vertices[l];
                    if (((a.y > p.y) != (b.y > p.y)) &&
                        (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) {
                        inside = !inside;
                    }
                }
                if (inside) {
                    occupied[i][j] = true;
                    distance_field_[i][j] = 0.0;
                }
            }
        }

        for (int i = 0; i < height_; ++i) {
            for (int j = 0; j < width_; ++j) {
                Point2D p = gridToWorld(i, j);
                double min_dist = distance_field_[i][j];
                int n = obstacle.vertices.size();
                for (int k = 0; k < n; ++k) {
                    const Point2D& a = obstacle.vertices[k];
                    const Point2D& b = obstacle.vertices[(k + 1) % n];
                    double dist = pointToSegmentDistance(p, a, b);
                    if (dist < min_dist) {
                        min_dist = dist;
                    }
                }
                distance_field_[i][j] = min_dist;
            }
        }
    }

    const double INF = std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> f(height_, std::vector<double>(width_, INF));
    for (int i = 0; i < height_; ++i) {
        for (int j = 0; j < width_; ++j) {
            if (occupied[i][j]) {
                f[i][j] = 0;
            }
        }
    }

    for (int i = 0; i < height_; ++i) {
        std::vector<double> g(width_, INF);
        for (int j = 0; j < width_; ++j) {
            g[j] = f[i][j];
        }
        std::vector<double> dist(width_, INF);
        std::deque<int> q;
        for (int j = 0; j < width_; ++j) {
            if (g[j] < INF) {
                dist[j] = 0;
                q.push_back(j);
            }
        }
        while (!q.empty()) {
            int j = q.front();
            q.pop_front();
            for (int dj : {-1, 1}) {
                int nj = j + dj;
                if (nj >= 0 && nj < width_) {
                    double new_dist = dist[j] + 1;
                    if (new_dist < dist[nj]) {
                        dist[nj] = new_dist;
                        q.push_back(nj);
                    }
                }
            }
        }
        for (int j = 0; j < width_; ++j) {
            f[i][j] = dist[j] * resolution_;
        }
    }

    for (int j = 0; j < width_; ++j) {
        std::vector<double> g(height_, INF);
        for (int i = 0; i < height_; ++i) {
            g[i] = f[i][j];
        }
        std::vector<double> dist(height_, INF);
        std::deque<int> q;
        for (int i = 0; i < height_; ++i) {
            if (g[i] < INF) {
                dist[i] = 0;
                q.push_back(i);
            }
        }
        while (!q.empty()) {
            int i = q.front();
            q.pop_front();
            for (int di : {-1, 1}) {
                int ni = i + di;
                if (ni >= 0 && ni < height_) {
                    double new_dist = dist[i] + 1;
                    if (new_dist < dist[ni]) {
                        dist[ni] = new_dist;
                        q.push_back(ni);
                    }
                }
            }
        }
        for (int i = 0; i < height_; ++i) {
            distance_field_[i][j] = std::min(distance_field_[i][j], dist[i] * resolution_);
        }
    }
}

double ESDF::getDistance(double x, double y) const {
    if (x < x_min_ || x > x_max_ || y < y_min_ || y > y_max_) {
        return std::numeric_limits<double>::infinity();
    }
    Point2D grid = worldToGrid(x, y);
    int i0 = static_cast<int>(std::floor(grid.x));
    int j0 = static_cast<int>(std::floor(grid.y));
    int i1 = i0 + 1;
    int j1 = j0 + 1;

    i0 = std::max(0, std::min(i0, height_ - 1));
    i1 = std::max(0, std::min(i1, height_ - 1));
    j0 = std::max(0, std::min(j0, width_ - 1));
    j1 = std::max(0, std::min(j1, width_ - 1));

    double dx = grid.y - j0;
    double dy = grid.x - i0;

    double d00 = distance_field_[i0][j0];
    double d01 = distance_field_[i0][j1];
    double d10 = distance_field_[i1][j0];
    double d11 = distance_field_[i1][j1];

    double d0 = d00 * (1 - dx) + d01 * dx;
    double d1 = d10 * (1 - dx) + d11 * dx;
    return d0 * (1 - dy) + d1 * dy;
}

Eigen::Vector2d ESDF::getGradient(double x, double y) const {
    double h = resolution_ * 0.5;
    double dx = (getDistance(x + h, y) - getDistance(x - h, y)) / (2 * h);
    double dy = (getDistance(x, y + h) - getDistance(x, y - h)) / (2 * h);
    return Eigen::Vector2d(dx, dy);
}

bool ESDF::isInCollision(double x, double y, double radius) const {
    return getDistance(x, y) < radius;
}

} // namespace motion_planner
