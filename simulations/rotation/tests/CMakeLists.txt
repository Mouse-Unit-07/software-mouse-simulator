#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for rotation test files                                  #
#                                                                              #
#------------------------------------------------------------------------------#
set_property(GLOBAL APPEND PROPERTY
    ALL_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_rotation.cpp
    ${CMAKE_SOURCE_DIR}/src/simulations/rotation/rotation.cpp
    ${CMAKE_SOURCE_DIR}/src/simulations/rotation/rotation.hpp
)

add_executable(test_rotation
    ${CMAKE_CURRENT_SOURCE_DIR}/test_rotation.cpp
    ${CMAKE_SOURCE_DIR}/platforms/windows/test_main.cpp
)

target_link_libraries(test_rotation PRIVATE
    rotation_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_rotation COMMAND test_rotation)
