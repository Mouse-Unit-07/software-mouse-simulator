#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for ray test files                                       #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all test and associated tested source files
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_ray.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/ray/ray.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/ray/ray.hpp
    PARENT_SCOPE
)

add_executable(test_ray # <=== Define your test target
    ${CMAKE_CURRENT_SOURCE_DIR}/test_ray.cpp # <=== Add all test source files
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_ray PRIVATE # <=== Rename to your test target
    ray_lib # <== Link to all libraries defined by your software project needed in the executable
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_ray COMMAND test_ray) # <=== Rename to your test target
