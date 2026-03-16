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
)

target_link_libraries(maze_lib PUBLIC
    point_lib
    rectangular_hitbox_lib
)
