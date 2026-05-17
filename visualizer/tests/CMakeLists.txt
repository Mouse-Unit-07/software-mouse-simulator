#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for visualizer test files                                #
#                                                                              #
#------------------------------------------------------------------------------#
set_property(GLOBAL APPEND PROPERTY
    ALL_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_visualizer.cpp
    ${CMAKE_SOURCE_DIR}/src/visualizer/visualizer.cpp
    ${CMAKE_SOURCE_DIR}/src/visualizer/visualizer.hpp
)

add_executable(test_visualizer
    ${CMAKE_CURRENT_SOURCE_DIR}/test_visualizer.cpp
    ${CMAKE_SOURCE_DIR}/platforms/windows/test_main.cpp
)

target_link_libraries(test_visualizer PRIVATE
    visualizer_lib
    CppUTest::CppUTest
    CppUTestExt
)

target_compile_definitions(test_visualizer PRIVATE SFML_STATIC)

add_test(NAME test_visualizer COMMAND test_visualizer)
