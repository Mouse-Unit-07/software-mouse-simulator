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

add_subdirectory(building_blocks/geometry/interactions)
add_subdirectory(building_blocks/geometry/point)
add_subdirectory(building_blocks/geometry/ray)
add_subdirectory(building_blocks/geometry/rectangular_hitbox)
add_subdirectory(building_blocks/maze)
add_subdirectory(building_blocks/mouse)
add_subdirectory(mock_device_drivers)
add_subdirectory(optimizer)
add_subdirectory(simulations/rotation)
add_subdirectory(visualizer)
