#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for move_forward_optimizer test files                    #
#                                                                              #
#------------------------------------------------------------------------------#
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_move_forward_optimizer.cpp
    ${CMAKE_SOURCE_DIR}/src/optimizer/move_forward/move_forward_optimizer.cpp
    ${CMAKE_SOURCE_DIR}/src/optimizer/move_forward/move_forward_optimizer.hpp
    PARENT_SCOPE
)

add_executable(test_move_forward_optimizer
    ${CMAKE_CURRENT_SOURCE_DIR}/test_move_forward_optimizer.cpp
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_move_forward_optimizer PRIVATE
    move_forward_optimizer_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_move_forward_optimizer COMMAND test_move_forward_optimizer)

add_custom_command(TARGET test_move_forward_optimizer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:pagmo>
        $<TARGET_FILE_DIR:test_move_forward_optimizer>
)
