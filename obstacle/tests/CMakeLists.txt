#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for obstacle test files                                  #
#                                                                              #
#------------------------------------------------------------------------------#
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_obstacle.cpp
    ${CMAKE_SOURCE_DIR}/src/obstacle/obstacle.cpp
    ${CMAKE_SOURCE_DIR}/src/obstacle/obstacle.hpp
    PARENT_SCOPE
)

add_executable(test_obstacle
    ${CMAKE_CURRENT_SOURCE_DIR}/test_obstacle.cpp
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_obstacle PRIVATE
    obstacle_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_obstacle COMMAND test_obstacle)
