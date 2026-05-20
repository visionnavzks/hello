cmake_minimum_required(VERSION 3.10)
project(TiledGridMap)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable optimization for release builds by default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Add executable
add_executable(tiled_grid_map_test main.cpp)

# Target include directories
target_include_directories(tiled_grid_map_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
