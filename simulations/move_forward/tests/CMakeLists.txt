#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for move_forward test files                              #
#                                                                              #
#------------------------------------------------------------------------------#
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_move_forward.cpp
    ${CMAKE_SOURCE_DIR}/src/simulations/move_forward/move_forward.cpp
    ${CMAKE_SOURCE_DIR}/src/simulations/move_forward/move_forward.hpp
    PARENT_SCOPE
)

add_executable(test_move_forward
    ${CMAKE_CURRENT_SOURCE_DIR}/test_move_forward.cpp
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_move_forward PRIVATE
    move_forward_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_move_forward COMMAND test_move_forward)
