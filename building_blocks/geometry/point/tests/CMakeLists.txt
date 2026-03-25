#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for point test files                                     #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all test and associated tested source files
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_point.cpp
    ${CMAKE_SOURCE_DIR}/src/building_blocks/geometry/point/point.cpp
    ${CMAKE_SOURCE_DIR}/src/building_blocks/geometry/point/point.hpp
    PARENT_SCOPE
)

add_executable(test_point # <=== Define your test target
    ${CMAKE_CURRENT_SOURCE_DIR}/test_point.cpp # <=== Add all test source files
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_point PRIVATE # <=== Rename to your test target
    point_lib # <== Link to all libraries defined by your software project needed in the executable
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_point COMMAND test_point) # <=== Rename to your test target
