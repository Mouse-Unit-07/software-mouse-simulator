#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for optimizer source files                               #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(optimizer_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/optimizer.cpp
)

target_include_directories(optimizer_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

include(FetchContent)

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

target_link_libraries(optimizer_lib PUBLIC
    pagmo
    Threads::Threads
    simulation_common_lib
    rotation_lib
)
