#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for visualizer test files                                #
#                                                                              #
#------------------------------------------------------------------------------#
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_visualizer.cpp
    ${CMAKE_SOURCE_DIR}/src/visualizer/visualizer.cpp
    ${CMAKE_SOURCE_DIR}/src/visualizer/visualizer.hpp
    PARENT_SCOPE
)

add_executable(test_visualizer
    ${CMAKE_CURRENT_SOURCE_DIR}/test_visualizer.cpp
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_visualizer PRIVATE
    visualizer_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_visualizer COMMAND test_visualizer)
