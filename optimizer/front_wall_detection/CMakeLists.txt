#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for front_wall_detection_optimizer source files          #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(front_wall_detection_optimizer_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/front_wall_detection_optimizer.cpp
)

target_include_directories(front_wall_detection_optimizer_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(front_wall_detection_optimizer_lib PUBLIC
    pagmo
    Threads::Threads
    simulation_common_lib
    optimizer_common_lib
    front_wall_detection_lib
)
