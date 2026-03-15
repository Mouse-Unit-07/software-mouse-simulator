#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for point source files                                   #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(point_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/point.cpp
)

target_include_directories(point_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)
