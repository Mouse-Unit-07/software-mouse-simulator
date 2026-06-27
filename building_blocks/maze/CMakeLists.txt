#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for maze builder source files                            #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(maze_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/maze.cpp
)

target_include_directories(maze_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    $<TARGET_PROPERTY:point_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:ray_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:interactions_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:rectangular_hitbox_lib,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(maze_lib PRIVATE
    point_lib
    ray_lib
    interactions_lib
    rectangular_hitbox_lib
)
