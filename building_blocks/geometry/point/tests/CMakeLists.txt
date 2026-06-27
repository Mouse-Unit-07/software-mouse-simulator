#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for point test files                                     #
#                                                                              #
#------------------------------------------------------------------------------#
set_property(GLOBAL APPEND PROPERTY
    ALL_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_point.cpp
    ${CMAKE_SOURCE_DIR}/src/building_blocks/geometry/point/point.cpp
    ${CMAKE_SOURCE_DIR}/src/building_blocks/geometry/point/point.hpp
)

add_executable(test_point
    ${CMAKE_CURRENT_SOURCE_DIR}/test_point.cpp
    ${CMAKE_SOURCE_DIR}/platforms/windows/test_main.cpp
)

target_link_libraries(test_point PRIVATE
    point_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_point COMMAND test_point)
