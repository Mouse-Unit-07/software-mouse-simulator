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
    $<TARGET_PROPERTY:pagmo,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:Threads::Threads,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:simulation_common_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:optimizer_common_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:side_wall_detection_lib,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(side_wall_detection_optimizer_lib PRIVATE
    pagmo
    Threads::Threads
    simulation_common_lib
    optimizer_common_lib
    side_wall_detection_lib
)
