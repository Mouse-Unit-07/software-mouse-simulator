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
