#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for move_forward_optimizer source files                  #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(move_forward_optimizer_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/move_forward_optimizer.cpp
)

target_include_directories(move_forward_optimizer_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    $<TARGET_PROPERTY:pagmo,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:Threads::Threads,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:simulation_common_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:optimizer_common_lib,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:move_forward_lib,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(move_forward_optimizer_lib PRIVATE
    pagmo
    Threads::Threads
    simulation_common_lib
    optimizer_common_lib
    move_forward_lib
)
