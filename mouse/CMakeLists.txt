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
)

target_link_libraries(mouse_lib PUBLIC
    point_lib
    ray_lib
    rectangular_hitbox_lib
)
