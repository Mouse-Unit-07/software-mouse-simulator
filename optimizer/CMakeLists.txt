#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for optimizer source files                               #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all .c files to your build target
add_library(optimizer_lib STATIC # <== Define this library's build target
    ${CMAKE_CURRENT_LIST_DIR}/optimizer.cpp
)

# \/=== Add all directories w/ header files
target_include_directories(optimizer_lib PUBLIC # <=== Rename to your build target
    ${CMAKE_CURRENT_LIST_DIR}
)

# \/=== Link to libraries in other GitHub projects
# Example dependency on Hello World project:
target_link_libraries(optimizer_lib PUBLIC
    mouse_lib
    obstacle_lib
    physics_lib
)
