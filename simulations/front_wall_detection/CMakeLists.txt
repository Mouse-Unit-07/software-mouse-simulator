#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for front_wall_detection source files                    #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(front_wall_detection_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/front_wall_detection.cpp
)

target_include_directories(front_wall_detection_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(front_wall_detection_lib PUBLIC
    mock_device_drivers_lib
    mouse_lib
    maze_lib
    simulation_common_lib
    visualizer_lib
)
