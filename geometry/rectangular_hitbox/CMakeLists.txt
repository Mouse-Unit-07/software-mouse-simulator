#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for rectangular_hitbox source files                      #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all .c files to your build target
add_library(rectangular_hitbox_lib STATIC # <== Define this library's build target
    ${CMAKE_CURRENT_LIST_DIR}/rectangular_hitbox.cpp
)

# \/=== Add all directories w/ header files
target_include_directories(rectangular_hitbox_lib PUBLIC # <=== Rename to your build target
    ${CMAKE_CURRENT_LIST_DIR}
)

# \/=== Link to libraries in other GitHub projects
# Example dependency on Hello World project:
target_link_libraries(rectangular_hitbox_lib PUBLIC
    point_lib
)
