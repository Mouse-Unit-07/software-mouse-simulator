#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for point source files                                   #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all .c files to your build target
add_library(point_lib STATIC # <== Define this library's build target
    ${CMAKE_CURRENT_LIST_DIR}/point.cpp
)

# \/=== Add all directories w/ header files
target_include_directories(point_lib PUBLIC # <=== Rename to your build target
    ${CMAKE_CURRENT_LIST_DIR}
)
