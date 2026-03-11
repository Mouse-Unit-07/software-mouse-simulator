#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# Top level CMakeLists.txt file (for this software repo)                       #
#                                                                              #
#------------------------------------------------------------------------------#
cmake_minimum_required(VERSION 4.0.2)
project(mouse_simulator_project LANGUAGES C CXX) # <== Name your software project

include(FetchContent)

# \/=== Add dependencies to other GitHub repos
# Example dependency on Hello World project:
FetchContent_Declare(
    device_drivers
    GIT_REPOSITORY git@github.com:Mouse-Unit-07/software-device-drivers.git
    GIT_TAG main
)
FetchContent_MakeAvailable(device_drivers)

# \/=== Add all library directories (each w/ a CMakeLists.txt)
add_subdirectory(mouse)
add_subdirectory(obstacle)
add_subdirectory(optimizer)
add_subdirectory(geometry/interactions)
add_subdirectory(geometry/point)
add_subdirectory(geometry/ray)
add_subdirectory(geometry/rectangular_hitbox)
