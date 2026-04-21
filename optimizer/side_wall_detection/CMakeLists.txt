#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for side_wall_detection_optimizer source files           #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(side_wall_detection_optimizer_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/side_wall_detection_optimizer.cpp
)

target_include_directories(side_wall_detection_optimizer_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(side_wall_detection_optimizer_lib PUBLIC
    pagmo
    Threads::Threads
    simulation_common_lib
    optimizer_common_lib
    side_wall_detection_lib
)
