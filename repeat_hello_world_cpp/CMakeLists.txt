#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for repeat_hello_world_cpp source files                  #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all .c files to your build target
add_library(repeat_hello_world_cpp_lib STATIC # <== Define this library's build target
    ${CMAKE_CURRENT_LIST_DIR}/repeat_hello_world_cpp.cpp
)

# \/=== Add all directories w/ header files
target_include_directories(repeat_hello_world_cpp_lib PUBLIC # <=== Rename to your build target
    ${CMAKE_CURRENT_LIST_DIR}
)

# \/=== Link to libraries in other GitHub projects
# Example dependency on Hello World project:
target_link_libraries(repeat_hello_world_cpp_lib PUBLIC
    hello_world_lib
)
