#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for interactions source files                            #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(interactions_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/interactions.cpp
)

target_include_directories(interactions_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    $<TARGET_PROPERTY:point_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:ray_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:rectangular_hitbox_lib,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(interactions_lib PRIVATE
    point_lib
    ray_lib
    rectangular_hitbox_lib
)
