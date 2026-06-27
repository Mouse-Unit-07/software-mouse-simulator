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
    $<TARGET_PROPERTY:mock_device_drivers_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:mouse_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:maze_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:simulation_common_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:visualizer_lib,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(rotation_lib PRIVATE
    mock_device_drivers_lib
    mouse_lib
    maze_lib
    simulation_common_lib
    visualizer_lib
)
