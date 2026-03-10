#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for mouse source files                                   #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all .c files to your build target
add_library(mouse_lib STATIC # <== Define this library's build target
    ${CMAKE_CURRENT_LIST_DIR}/mouse.cpp
)

# \/=== Add all directories w/ header files
target_include_directories(mouse_lib PUBLIC # <=== Rename to your build target
    ${CMAKE_CURRENT_LIST_DIR}
)
