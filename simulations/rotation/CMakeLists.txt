#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for rotation source files                                #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(rotation_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/rotation.cpp
)

target_include_directories(rotation_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(rotation_lib PUBLIC
    mock_device_drivers_lib
    mouse_lib
    maze_lib
    simulation_common_lib
    visualizer_lib
)
