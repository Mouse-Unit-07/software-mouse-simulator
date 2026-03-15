#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for optimizer source files                               #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(optimizer_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/optimizer.cpp
)

target_include_directories(optimizer_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(optimizer_lib PUBLIC
    mouse_lib
    obstacle_lib
    interactions_lib
)
