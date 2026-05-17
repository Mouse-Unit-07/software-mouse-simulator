#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for ray test files                                       #
#                                                                              #
#------------------------------------------------------------------------------#
set_property(GLOBAL APPEND PROPERTY
    ALL_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_ray.cpp
    ${CMAKE_SOURCE_DIR}/src/building_blocks/geometry/ray/ray.cpp
    ${CMAKE_SOURCE_DIR}/src/building_blocks/geometry/ray/ray.hpp
)

add_executable(test_ray
    ${CMAKE_CURRENT_SOURCE_DIR}/test_ray.cpp
    ${CMAKE_SOURCE_DIR}/platforms/windows/test_main.cpp
)

target_link_libraries(test_ray PRIVATE
    ray_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_ray COMMAND test_ray)
