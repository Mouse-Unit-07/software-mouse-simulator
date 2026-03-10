#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for obstacle test files                                  #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all test and associated tested source files
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_obstacle.cpp
    ${CMAKE_SOURCE_DIR}/src/obstacle/obstacle.cpp
    ${CMAKE_SOURCE_DIR}/src/obstacle/obstacle.hpp
    PARENT_SCOPE
)

add_executable(test_obstacle # <=== Define your test target
    ${CMAKE_CURRENT_SOURCE_DIR}/test_obstacle.cpp # <=== Add all test source files
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_obstacle PRIVATE # <=== Rename to your test target
    obstacle_lib # <== Link to all libraries defined by your software project needed in the executable
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_obstacle COMMAND test_obstacle) # <=== Rename to your test target
