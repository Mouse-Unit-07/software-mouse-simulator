#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for obstacle source files                               #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all .c files to your build target
add_library(obstacle_lib STATIC # <== Define this library's build target
    ${CMAKE_CURRENT_LIST_DIR}/obstacle.cpp
)

# \/=== Add all directories w/ header files
target_include_directories(obstacle_lib PUBLIC # <=== Rename to your build target
    ${CMAKE_CURRENT_LIST_DIR}
)

# \/=== Link to libraries in other GitHub projects
# Example dependency on Hello World project:
target_link_libraries(obstacle_lib PUBLIC
    point_lib
    rectangular_hitbox_lib
)
