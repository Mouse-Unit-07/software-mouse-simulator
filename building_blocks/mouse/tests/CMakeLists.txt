#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for mouse test files                                     #
#                                                                              #
#------------------------------------------------------------------------------#
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_mouse.cpp
    ${CMAKE_SOURCE_DIR}/src/building_blocks/mouse/mouse.cpp
    ${CMAKE_SOURCE_DIR}/src/building_blocks/mouse/mouse.hpp
    PARENT_SCOPE
)

add_executable(test_mouse
    ${CMAKE_CURRENT_SOURCE_DIR}/test_mouse.cpp
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_mouse PRIVATE
    mouse_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_mouse COMMAND test_mouse)
