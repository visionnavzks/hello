#ifndef TILED_DISTANCE_FIELD_H
#define TILED_DISTANCE_FIELD_H

#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <cassert>
#include <algorithm>

// 地图常数配置
const float MAX_DIST = 3.0f;     // 截断距离 3米
const int BLOCK_BITS = 6;        // 2^6 = 64
const int BLOCK_SIZE = 1 << BLOCK_BITS; // 64
const int BLOCK_MASK = BLOCK_SIZE - 1;  // 63，用于位运算取模

// 单个分块：内存绝对连续，极大提升 CPU 缓存命中率
struct GridBlock {
    float distance[BLOCK_SIZE][BLOCK_SIZE];

    GridBlock() {
        // 初始化时，默认所有点都在 3 米之外
        for (int i = 0; i < BLOCK_SIZE; ++i) {
            for (int j = 0; j < BLOCK_SIZE; ++j) {
                distance[i][j] = MAX_DIST;
            }
        }
    }
};

class TiledDistanceField {
public:
    TiledDistanceField(double min_x, double min_y, double max_x, double max_y, double resolution)
        : min_x_(min_x), min_y_(min_y), res_(resolution) {

        // 计算总网格数
        int total_cells_x = static_cast<int>(std::ceil((max_x - min_x) / res_));
        int total_cells_y = static_cast<int>(std::ceil((max_y - min_y) / res_));

        // 计算需要多少个 Block
        num_blocks_x_ = (total_cells_x + BLOCK_SIZE - 1) >> BLOCK_BITS;
        num_blocks_y_ = (total_cells_y + BLOCK_SIZE - 1) >> BLOCK_BITS;

        // 分配全局指针数组（仅占几兆内存，存的都是空指针）
        blocks_.resize(num_blocks_x_ * num_blocks_y_);
    }

    ~TiledDistanceField() = default;

    // 禁止拷贝，防止指针重复释放
    TiledDistanceField(const TiledDistanceField&) = delete;
    TiledDistanceField& operator=(const TiledDistanceField&) = delete;

    /**
     * 【写入方法】：仅在构建地图时调用
     */
    void setDistance(double world_x, double world_y, float dist) {
        if (dist >= MAX_DIST) return; // 超过3米，不分配内存，直接丢弃

        int gx = static_cast<int>((world_x - min_x_) / res_);
        int gy = static_cast<int>((world_y - min_y_) / res_);

        int bx = gx >> BLOCK_BITS;
        int by = gy >> BLOCK_BITS;

        if (bx < 0 || bx >= num_blocks_x_ || by < 0 || by >= num_blocks_y_) return;

        size_t block_idx = bx * num_blocks_y_ + by;
        if (!blocks_[block_idx]) {
            // 只有路径经过的 3 米范围内，才会真正触发内存分配
            blocks_[block_idx] = std::make_unique<GridBlock>();
        }

        int local_x = gx & BLOCK_MASK;
        int local_y = gy & BLOCK_MASK;
        blocks_[block_idx]->distance[local_x][local_y] = std::min(blocks_[block_idx]->distance[local_x][local_y], dist);
    }

    /**
     * 【核心读取方法】：支持双线性插值，输出距离与解析梯度
     * 用于优化器的目标函数迭代，纯粹的 O(1) 速度
     */
    bool getDistanceAndGradient(double world_x, double world_y, float& dist, float& grad_x, float& grad_y) const {
        // 1. 转换到连续网格坐标
        double fx = (world_x - min_x_) / res_;
        double fy = (world_y - min_y_) / res_;

        // 2. 找到左下角最近的整数像素点
        int x0 = static_cast<int>(std::floor(fx));
        int y0 = static_cast<int>(std::floor(fy));

        // 3. 计算插值权重 (0 到 1 之间)
        float tx = fx - x0;
        float ty = fy - y0;

        // 4. 读取相邻的 4 个网格点值（通过位运算寻址，极快）
        float g00 = getCellValue(x0, y0);
        float g10 = getCellValue(x0 + 1, y0);
        float g01 = getCellValue(x0, y0 + 1);
        float g11 = getCellValue(x0 + 1, y0 + 1);

        // 5. 双线性插值计算距离
        dist = (1.0f - tx) * (1.0f - ty) * g00 +
               tx * (1.0f - ty) * g10 +
               (1.0f - tx) * ty * g01 +
               tx * ty * g11;

        // 6. 计算关于网格的偏导数，并通过链式法则乘以 (1/res) 转化为世界坐标系梯度
        float d_dfx = (1.0f - ty) * (g10 - g00) + ty * (g11 - g01);
        float d_dfy = (1.0f - tx) * (g01 - g00) + tx * (g11 - g10);

        grad_x = d_dfx / static_cast<float>(res_);
        grad_y = d_dfy / static_cast<float>(res_);

        return true;
    }

private:
    // 内联基础读取：无缝处理跨 Block 边界查询
    inline float getCellValue(int gx, int gy) const {
        int bx = gx >> BLOCK_BITS;
        int by = gy >> BLOCK_BITS;

        if (bx < 0 || bx >= num_blocks_x_ || by < 0 || by >= num_blocks_y_) {
            return MAX_DIST;
        }

        size_t block_idx = bx * num_blocks_y_ + by;
        GridBlock* b = blocks_[block_idx].get();

        // 如果该分块未分配内存，说明它在 3 米之外
        if (!b) return MAX_DIST;

        return b->distance[gx & BLOCK_MASK][gy & BLOCK_MASK];
    }

    double min_x_, min_y_;
    double res_;
    int num_blocks_x_;
    int num_blocks_y_;
    std::vector<std::unique_ptr<GridBlock>> blocks_; // 平铺的指针一维数组
};

#endif // TILED_DISTANCE_FIELD_H
