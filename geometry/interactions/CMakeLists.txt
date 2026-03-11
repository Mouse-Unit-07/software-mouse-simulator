#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for interactions source files                            #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all .c files to your build target
add_library(interactions_lib STATIC # <== Define this library's build target
    ${CMAKE_CURRENT_LIST_DIR}/interactions.cpp
)

# \/=== Add all directories w/ header files
target_include_directories(interactions_lib PUBLIC # <=== Rename to your build target
    ${CMAKE_CURRENT_LIST_DIR}
)
