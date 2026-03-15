#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for obstacle source files                               #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(obstacle_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/obstacle.cpp
)

target_include_directories(obstacle_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(obstacle_lib PUBLIC
    point_lib
    rectangular_hitbox_lib
)
