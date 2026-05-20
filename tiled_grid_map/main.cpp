#include "TiledDistanceField.h"
#include "PathBuilder.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <random>

void runTests() {
    std::cout << "Starting tests..." << std::endl;

    double map_min_x = 0.0, map_min_y = 0.0;
    double resolution = 0.05; // 5 cm resolution

    // Initialize an empty map
    TiledDistanceField sdf(map_min_x, map_min_y, 1000.0, 1000.0, resolution);

    // Path representing a straight line along the x-axis
    std::vector<Point> path = {
        {10.0, 10.0}, {20.0, 10.0}, {30.0, 10.0}
    };

    // Build the distance field from the path
    buildDistanceFieldFromPath(sdf, path, resolution, map_min_x, map_min_y);

    // Test querying exactly on the path (should be close to 0)
    float dist, gx, gy;
    sdf.getDistanceAndGradient(15.0, 10.0, dist, gx, gy);
    std::cout << "Test 1 (On path): dist = " << dist << ", grad = (" << gx << ", " << gy << ")\n";
    if (dist > 0.1f) {
        std::cerr << "Test 1 failed: distance too large on path!\n";
    }

    // Test querying slightly off the path along y-axis
    // The closest point is (15, 10), so the distance should be about 1.0
    // The gradient should point away from the path, so mostly along y.
    sdf.getDistanceAndGradient(15.0, 11.0, dist, gx, gy);
    std::cout << "Test 2 (Off path, y+): dist = " << dist << ", grad = (" << gx << ", " << gy << ")\n";
    if (std::abs(dist - 1.0f) > 0.1f) {
        std::cerr << "Test 2 failed: distance mismatch!\n";
    }
    if (gy < 0.8f) { // Gradient should be mostly pointing positive y (away from line y=10)
        std::cerr << "Test 2 failed: incorrect gradient direction!\n";
    }

    // Test querying slightly off the path along negative y-axis
    sdf.getDistanceAndGradient(15.0, 9.0, dist, gx, gy);
    std::cout << "Test 3 (Off path, y-): dist = " << dist << ", grad = (" << gx << ", " << gy << ")\n";
    if (std::abs(dist - 1.0f) > 0.1f) {
        std::cerr << "Test 3 failed: distance mismatch!\n";
    }
    // Gradient points away from line y=10 towards y=9, so gradient is negative y.
    // Actually, wait: gradient points in direction of increasing distance!
    // At y=9, moving to y=8 increases distance. So gradient should be negative y.
    if (gy > -0.8f) {
        std::cerr << "Test 3 failed: incorrect gradient direction!\n";
    }

    // Test far away (unallocated block)
    sdf.getDistanceAndGradient(500.0, 500.0, dist, gx, gy);
    std::cout << "Test 4 (Far away): dist = " << dist << ", grad = (" << gx << ", " << gy << ")\n";
    if (dist != MAX_DIST) {
        std::cerr << "Test 4 failed: unallocated block did not return MAX_DIST!\n";
    }
    if (gx != 0.0f || gy != 0.0f) {
        std::cerr << "Test 4 failed: gradient should be zero in flat MAX_DIST area!\n";
    }

    // Original demo from user
    std::cout << "\nRunning user demo..." << std::endl;
    std::vector<Point> demo_path = {
        {10.0, 10.0}, {20.0, 15.0}, {30.0, 12.0}, {50.0, 40.0}
    };
    TiledDistanceField demo_sdf(map_min_x, map_min_y, 1000.0, 1000.0, resolution);
    buildDistanceFieldFromPath(demo_sdf, demo_path, resolution, map_min_x, map_min_y);
    demo_sdf.getDistanceAndGradient(22.0, 14.5, dist, gx, gy);
    std::cout << "User demo result: dist = " << dist << ", grad = (" << gx << ", " << gy << ")\n";

    std::cout << "Tests finished." << std::endl;
}

void runBenchmark() {
    std::cout << "\nStarting Performance Benchmark..." << std::endl;

    double map_min_x = 0.0, map_min_y = 0.0;
    double resolution = 0.05;
    TiledDistanceField sdf(map_min_x, map_min_y, 1000.0, 1000.0, resolution);

    // Create a long dummy path
    std::vector<Point> path;
    for (int i = 0; i < 1000; ++i) {
        path.push_back({10.0 + i * 0.5, 10.0 + i * 0.5});
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    buildDistanceFieldFromPath(sdf, path, resolution, map_min_x, map_min_y);
    auto t2 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> build_ms = t2 - t1;
    std::cout << "Built distance field for path of length " << path.size() << " in " << build_ms.count() << " ms." << std::endl;

    // Benchmark Random Access
    const int NUM_QUERIES = 10000000; // 10 million queries

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist_x(0.0, 1000.0);
    std::uniform_real_distribution<double> dist_y(0.0, 1000.0);

    // Pre-generate random coordinates to exclude RNG overhead from benchmark
    std::vector<Point> queries(NUM_QUERIES);
    for (int i = 0; i < NUM_QUERIES; ++i) {
        queries[i] = {dist_x(rng), dist_y(rng)};
    }

    float dist, gx, gy;
    float dummy_sum = 0.0f; // to prevent optimizer from stripping out the loop

    auto t3 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NUM_QUERIES; ++i) {
        sdf.getDistanceAndGradient(queries[i].x, queries[i].y, dist, gx, gy);
        dummy_sum += dist;
    }
    auto t4 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> query_ms = t4 - t3;
    std::cout << "Executed " << NUM_QUERIES << " random getDistanceAndGradient queries in " << query_ms.count() << " ms." << std::endl;
    std::cout << "Average time per query: " << (query_ms.count() * 1e6 / NUM_QUERIES) << " nanoseconds." << std::endl;

    // Print dummy sum so it isn't optimized away
    std::cout << "(Dummy sum check: " << dummy_sum << ")" << std::endl;
}

int main() {
    runTests();
    runBenchmark();
    return 0;
}
