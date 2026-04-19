#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# Top level CMakeLists.txt file (for this software repo)                       #
#                                                                              #
#------------------------------------------------------------------------------#
cmake_minimum_required(VERSION 4.0.2)
project(mouse_simulator_project LANGUAGES C CXX)

include(FetchContent)

FetchContent_Declare(
    device_drivers
    GIT_REPOSITORY git@github.com:Mouse-Unit-07/software-device-drivers.git
    GIT_TAG main
)
FetchContent_MakeAvailable(device_drivers)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(PAGMO_BUILD_STATIC_LIBS ON CACHE BOOL "" FORCE)
set(PAGMO_ENABLE_TBB OFF CACHE BOOL "" FORCE)
set(PAGMO_WITH_TBB OFF CACHE BOOL "" FORCE)
set(PAGMO_WITH_EIGEN3 OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    pagmo2
    GIT_REPOSITORY https://github.com/esa/pagmo2.git
    GIT_TAG        v2.19.0
)
FetchContent_MakeAvailable(pagmo2)

find_package(Threads REQUIRED)

add_subdirectory(building_blocks/geometry/interactions)
add_subdirectory(building_blocks/geometry/point)
add_subdirectory(building_blocks/geometry/ray)
add_subdirectory(building_blocks/geometry/rectangular_hitbox)
add_subdirectory(building_blocks/maze)
add_subdirectory(building_blocks/mouse)
add_subdirectory(mock_device_drivers)
add_subdirectory(optimizer/common)
add_subdirectory(optimizer/move_forward)
add_subdirectory(optimizer/rotation)
add_subdirectory(simulations/common)
add_subdirectory(simulations/front_wall_detection)
add_subdirectory(simulations/move_forward)
add_subdirectory(simulations/rotation)
add_subdirectory(simulations/side_wall_detection)
add_subdirectory(visualizer)
