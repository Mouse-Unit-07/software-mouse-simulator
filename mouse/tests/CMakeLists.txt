#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for mouse test files                                     #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all test and associated tested source files
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_mouse.cpp
    ${CMAKE_SOURCE_DIR}/src/mouse/mouse.cpp
    ${CMAKE_SOURCE_DIR}/src/mouse/mouse.hpp
    PARENT_SCOPE
)

add_executable(test_mouse # <=== Define your test target
    ${CMAKE_CURRENT_SOURCE_DIR}/test_mouse.cpp # <=== Add all test source files
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_mouse PRIVATE # <=== Rename to your test target
    mouse_lib # <== Link to all libraries defined by your software project needed in the executable
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_mouse COMMAND test_mouse) # <=== Rename to your test target
