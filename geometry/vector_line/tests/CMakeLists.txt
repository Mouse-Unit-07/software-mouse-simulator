#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for vector_line test files                               #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all test and associated tested source files
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_vector_line.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/vector_line/vector_line.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/vector_line/vector_line.hpp
    PARENT_SCOPE
)

add_executable(test_vector_line # <=== Define your test target
    ${CMAKE_CURRENT_SOURCE_DIR}/test_vector_line.cpp # <=== Add all test source files
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_vector_line PRIVATE # <=== Rename to your test target
    vector_line_lib # <== Link to all libraries defined by your software project needed in the executable
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_vector_line COMMAND test_vector_line) # <=== Rename to your test target
