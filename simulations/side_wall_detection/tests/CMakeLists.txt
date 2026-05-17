#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for side_wall_detection test files                       #
#                                                                              #
#------------------------------------------------------------------------------#
set_property(GLOBAL APPEND PROPERTY
    ALL_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_side_wall_detection.cpp
    ${CMAKE_SOURCE_DIR}/src/simulations/side_wall_detection/side_wall_detection.cpp
    ${CMAKE_SOURCE_DIR}/src/simulations/side_wall_detection/side_wall_detection.hpp
)

add_executable(test_side_wall_detection
    ${CMAKE_CURRENT_SOURCE_DIR}/test_side_wall_detection.cpp
    ${CMAKE_SOURCE_DIR}/platforms/windows/test_main.cpp
)

target_link_libraries(test_side_wall_detection PRIVATE
    side_wall_detection_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_side_wall_detection COMMAND test_side_wall_detection)
