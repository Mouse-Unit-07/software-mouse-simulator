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
)

target_link_libraries(interactions_lib PUBLIC
    point_lib
    ray_lib
    rectangular_hitbox_lib
)
