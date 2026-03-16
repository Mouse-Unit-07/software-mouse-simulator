#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for visualizer source files                              #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(visualizer_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/visualizer.cpp
)

target_include_directories(visualizer_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(visualizer_lib PUBLIC
    point_lib
    ray_lib
    rectangular_hitbox_lib
    mouse_lib
    maze_lib
)


include(FetchContent)

set(SFML_BUILD_AUDIO OFF CACHE BOOL "" FORCE)
set(SFML_BUILD_NETWORK OFF CACHE BOOL "" FORCE)
set(SFML_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SFML_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(SFML_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    SFML
    GIT_REPOSITORY git@github.com:SFML/SFML.git
    GIT_TAG 2.6.1
)
FetchContent_MakeAvailable(SFML)

target_link_libraries(visualizer_lib PUBLIC
    sfml-graphics
    sfml-window
    sfml-system
)
