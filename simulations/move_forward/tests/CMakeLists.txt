#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for move_forward test files                              #
#                                                                              #
#------------------------------------------------------------------------------#
set_property(GLOBAL APPEND PROPERTY
    ALL_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_move_forward.cpp
    ${CMAKE_SOURCE_DIR}/src/simulations/move_forward/move_forward.cpp
    ${CMAKE_SOURCE_DIR}/src/simulations/move_forward/move_forward.hpp
)

add_executable(test_move_forward
    ${CMAKE_CURRENT_SOURCE_DIR}/test_move_forward.cpp
    ${CMAKE_SOURCE_DIR}/platforms/windows/test_main.cpp
)

target_link_libraries(test_move_forward PRIVATE
    move_forward_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_move_forward COMMAND test_move_forward)
