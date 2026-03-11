#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for vector_line source files                             #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all .c files to your build target
add_library(vector_line_lib STATIC # <== Define this library's build target
    ${CMAKE_CURRENT_LIST_DIR}/vector_line.cpp
)

# \/=== Add all directories w/ header files
target_include_directories(vector_line_lib PUBLIC # <=== Rename to your build target
    ${CMAKE_CURRENT_LIST_DIR}
)

# \/=== Link to libraries in other GitHub projects
# Example dependency on Hello World project:
target_link_libraries(vector_line_lib PUBLIC
    point_lib
)
