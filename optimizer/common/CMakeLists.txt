#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for optimizer_common source files                        #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(optimizer_common_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/optimizer_common.cpp
)

target_include_directories(optimizer_common_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)
