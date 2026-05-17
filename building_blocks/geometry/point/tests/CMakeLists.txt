#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for point test files                                     #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all test and associated tested source files
set_property(GLOBAL APPEND PROPERTY
    ALL_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_point.cpp
    ${CMAKE_SOURCE_DIR}/src/building_blocks/geometry/point/point.cpp
    ${CMAKE_SOURCE_DIR}/src/building_blocks/geometry/point/point.hpp
)

add_executable(test_point # <=== Define your test target
    ${CMAKE_CURRENT_SOURCE_DIR}/test_point.cpp # <=== Add all test source files
    ${CMAKE_SOURCE_DIR}/platforms/windows/test_main.cpp
)

target_link_libraries(test_point PRIVATE # <=== Rename to your test target
    point_lib # <== Link to all libraries defined by your software project needed in the executable
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_point COMMAND test_point) # <=== Rename to your test target
