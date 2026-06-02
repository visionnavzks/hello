cmake_minimum_required(VERSION 3.10)
project(motion_planner)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

add_library(motion_planner_lib
    src/esdf.cpp
    src/astar.cpp
    src/shortcut_smoother.cpp
    src/esdf_grad_smoother.cpp
    src/reeds_shepp.cpp
    src/se2_smoother.cpp
    src/robot_contour.cpp
    src/trajectory_validator.cpp
)

target_link_libraries(motion_planner_lib
    ${CERES_LIBRARIES}
)

add_executable(demo src/main.cpp)
target_link_libraries(demo motion_planner_lib)
