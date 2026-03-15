#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for ray source files                                     #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(ray_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/ray.cpp
)

target_include_directories(ray_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(ray_lib PUBLIC
    point_lib
)
