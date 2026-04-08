#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for front_wall_detection test files                       #
#                                                                              #
#------------------------------------------------------------------------------#
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_front_wall_detection.cpp
    ${CMAKE_SOURCE_DIR}/src/simulations/front_wall_detection/front_wall_detection.cpp
    ${CMAKE_SOURCE_DIR}/src/simulations/front_wall_detection/front_wall_detection.hpp
    PARENT_SCOPE
)

add_executable(test_front_wall_detection
    ${CMAKE_CURRENT_SOURCE_DIR}/test_front_wall_detection.cpp
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_front_wall_detection PRIVATE
    front_wall_detection_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_front_wall_detection COMMAND test_front_wall_detection)
