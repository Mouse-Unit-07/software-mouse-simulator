#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for maze builder test files                              #
#                                                                              #
#------------------------------------------------------------------------------#
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_maze.cpp
    ${CMAKE_SOURCE_DIR}/src/building_blocks/maze/maze.cpp
    ${CMAKE_SOURCE_DIR}/src/building_blocks/maze/maze.hpp
    PARENT_SCOPE
)

add_executable(test_maze
    ${CMAKE_CURRENT_SOURCE_DIR}/test_maze.cpp
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_maze PRIVATE
    mouse_lib
    maze_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_maze COMMAND test_maze)
