#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for mouse source files                                   #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(mouse_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/mouse.cpp
)

target_include_directories(mouse_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    $<TARGET_PROPERTY:point_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:ray_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:rectangular_hitbox_lib,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(mouse_lib PRIVATE
    point_lib
    ray_lib
    rectangular_hitbox_lib
)
