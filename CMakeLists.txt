#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# Top level CMakeLists.txt file (for this software repo)                       #
#                                                                              #
#------------------------------------------------------------------------------#
cmake_minimum_required(VERSION 4.0.2)
project(repeat_hello_world_cpp_project LANGUAGES C CXX) # <== Name your software project

include(FetchContent)

# \/=== Add dependencies to other GitHub repos
# Example dependency on Hello World project:
FetchContent_Declare(
    print_hello_world
    GIT_REPOSITORY git@github.com:Mouse-Unit-07/experiment-software-hello-world.git
    GIT_TAG main
)
FetchContent_MakeAvailable(print_hello_world)

# \/=== Add all library directories (each w/ a CMakeLists.txt)
add_subdirectory(repeat_hello_world_cpp)
