#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for rectangular_hitbox test files                        #
#                                                                              #
#------------------------------------------------------------------------------#
set_property(GLOBAL APPEND PROPERTY
    ALL_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_rectangular_hitbox.cpp
    ${CMAKE_SOURCE_DIR}/src/building_blocks/geometry/rectangular_hitbox/rectangular_hitbox.cpp
    ${CMAKE_SOURCE_DIR}/src/building_blocks/geometry/rectangular_hitbox/rectangular_hitbox.hpp
)

add_executable(test_rectangular_hitbox
    ${CMAKE_CURRENT_SOURCE_DIR}/test_rectangular_hitbox.cpp
    ${CMAKE_SOURCE_DIR}/platforms/windows/test_main.cpp
)

target_link_libraries(test_rectangular_hitbox PRIVATE
    rectangular_hitbox_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_rectangular_hitbox COMMAND test_rectangular_hitbox)
