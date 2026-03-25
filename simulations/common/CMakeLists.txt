#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for simulation_common source files                       #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(simulation_common_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/simulation_common.cpp
)

target_include_directories(simulation_common_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)
