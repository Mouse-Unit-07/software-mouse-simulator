#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for physics source files                                 #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all .c files to your build target
add_library(physics_lib STATIC # <== Define this library's build target
    ${CMAKE_CURRENT_LIST_DIR}/physics.cpp
)

# \/=== Add all directories w/ header files
target_include_directories(physics_lib PUBLIC # <=== Rename to your build target
    ${CMAKE_CURRENT_LIST_DIR}
)
