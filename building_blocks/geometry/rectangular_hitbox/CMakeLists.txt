#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for rectangular_hitbox source files                      #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(rectangular_hitbox_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/rectangular_hitbox.cpp
)

target_include_directories(rectangular_hitbox_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    $<TARGET_PROPERTY:point_lib,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(rectangular_hitbox_lib PRIVATE
    point_lib
)
