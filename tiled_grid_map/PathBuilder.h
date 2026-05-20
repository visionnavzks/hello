#ifndef PATH_BUILDER_H
#define PATH_BUILDER_H

#include "TiledDistanceField.h"
#include <vector>
#include <cmath>
#include <algorithm>

struct Point {
    double x, y;
};

// 辅助几何函数：计算点 P 到线段 AB 的最短距离
inline double computePointToSegmentDistance(const Point& p, const Point& a, const Point& b) {
    double ab_x = b.x - a.x;
    double ab_y = b.y - a.y;
    double ap_x = p.x - a.x;
    double ap_y = p.y - a.y;

    // 计算线段 AB 长度的平方
    double ab_len_sq = ab_x * ab_x + ab_y * ab_y;

    // 如果 A 和 B 重合，直接计算点到点距离
    if (ab_len_sq < 1e-6) {
        return std::hypot(ap_x, ap_y);
    }

    // 计算投影系数 t，并截断到 [0, 1] 区间（确保投影在线段内）
    double t = (ap_x * ab_x + ap_y * ab_y) / ab_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    // 找到最近点 C 的坐标
    double closest_x = a.x + t * ab_x;
    double closest_y = a.y + t * ab_y;

    // 返回 P 到最近点 C 的距离
    return std::hypot(p.x - closest_x, p.y - closest_y);
}

/**
 * 【核心构建函数】：高效生成 3 米窄带距离场
 * @param sdf  刚才定义的地图对象
 * @param path 连续的路径点集合 (S很长)
 */
inline void buildDistanceFieldFromPath(TiledDistanceField& sdf, const std::vector<Point>& path, double res, double min_x, double min_y) {
    if (path.size() < 2) return;

    const double SEARCH_RAD = MAX_DIST; // 只需要3米范围 (MAX_DIST is 3.0f defined in TiledDistanceField.h)

    // 遍历路径上的每一段线段 (A -> B)
    for (size_t i = 0; i < path.size() - 1; ++i) {
        const Point& A = path[i];
        const Point& B = path[i + 1];

        // 1. 关键优化：计算当前线段的轴向包围盒（AABB），并向外膨胀 3 米
        double seg_min_x = std::min(A.x, B.x) - SEARCH_RAD;
        double seg_max_x = std::max(A.x, B.x) + SEARCH_RAD;
        double seg_min_y = std::min(A.y, B.y) - SEARCH_RAD;
        double seg_max_y = std::max(A.y, B.y) + SEARCH_RAD;

        // 2. 将物理世界的包围盒转换成网格索引范围
        int min_gx = static_cast<int>(std::floor((seg_min_x - min_x) / res));
        int max_gx = static_cast<int>(std::ceil((seg_max_x - min_x) / res));
        int min_gy = static_cast<int>(std::floor((seg_min_y - min_y) / res));
        int max_gy = static_cast<int>(std::ceil((seg_max_y - min_y) / res));

        // 3. 仅在这个局部的【窄带矩形窗口】内进行迭代
        for (int gx = min_gx; gx <= max_gx; ++gx) {
            for (int gy = min_gy; gy <= max_gy; ++gy) {

                // 将当前网格中心点的物理坐标算出来
                Point cell_p;
                cell_p.x = min_x + (gx + 0.5) * res;
                cell_p.y = min_y + (gy + 0.5) * res;

                // 4. 计算该网格点到当前线段的最短几何距离
                double dist = computePointToSegmentDistance(cell_p, A, B);

                // 5. 如果距离在 3 米以内，写入地图（内部会自动分配稀疏内存，并保持最小值）
                if (dist < SEARCH_RAD) {
                    sdf.setDistance(cell_p.x, cell_p.y, static_cast<float>(dist));
                }
            }
        }
    }
}

#endif // PATH_BUILDER_H
