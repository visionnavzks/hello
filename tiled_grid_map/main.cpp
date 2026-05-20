#include "TiledDistanceField.h"
#include "PathBuilder.h"
#include <iostream>
#include <vector>
#include <cmath>

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

int main() {
    runTests();
    return 0;
}
