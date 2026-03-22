#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for rotation test files                                  #
#                                                                              #
#------------------------------------------------------------------------------#
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_rotation.cpp
    ${CMAKE_SOURCE_DIR}/src/simulations/rotation/rotation.cpp
    ${CMAKE_SOURCE_DIR}/src/simulations/rotation/rotation.hpp
    PARENT_SCOPE
)

add_executable(test_rotation
    ${CMAKE_CURRENT_SOURCE_DIR}/test_rotation.cpp
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_rotation PRIVATE
    rotation_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_rotation COMMAND test_rotation)
