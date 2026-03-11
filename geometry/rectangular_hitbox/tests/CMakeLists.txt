#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for rectangular_hitbox test files                        #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all test and associated tested source files
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_rectangular_hitbox.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/rectangular_hitbox/rectangular_hitbox.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/rectangular_hitbox/rectangular_hitbox.hpp
    PARENT_SCOPE
)

add_executable(test_rectangular_hitbox # <=== Define your test target
    ${CMAKE_CURRENT_SOURCE_DIR}/test_rectangular_hitbox.cpp # <=== Add all test source files
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_rectangular_hitbox PRIVATE # <=== Rename to your test target
    rectangular_hitbox_lib # <== Link to all libraries defined by your software project needed in the executable
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_rectangular_hitbox COMMAND test_rectangular_hitbox) # <=== Rename to your test target
