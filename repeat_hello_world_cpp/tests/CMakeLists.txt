#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for repeat_hello_world_cpp test files                    #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all test and associated tested source files
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_repeat_hello_world_cpp.cpp
    ${CMAKE_SOURCE_DIR}/src/repeat_hello_world_cpp/repeat_hello_world_cpp.cpp
    ${CMAKE_SOURCE_DIR}/src/repeat_hello_world_cpp/repeat_hello_world_cpp.hpp
    PARENT_SCOPE
)

add_executable(test_repeat_hello_world_cpp # <=== Define your test target
    ${CMAKE_CURRENT_SOURCE_DIR}/test_repeat_hello_world_cpp.cpp # <=== Add all test source files
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_repeat_hello_world_cpp PRIVATE # <=== Rename to your test target
    repeat_hello_world_cpp_lib # <== Link to all libraries defined by your software project needed in the executable
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_repeat_hello_world_cpp COMMAND test_repeat_hello_world_cpp) # <=== Rename to your test target
