#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for side_wall_detection source files                     #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(side_wall_detection_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/side_wall_detection.cpp
)

target_include_directories(side_wall_detection_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    $<TARGET_PROPERTY:mock_device_drivers_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:mouse_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:maze_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:simulation_common_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:visualizer_lib,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(side_wall_detection_lib PRIVATE
    mock_device_drivers_lib
    mouse_lib
    maze_lib
    simulation_common_lib
    visualizer_lib
)
