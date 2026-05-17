#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for side_wall_detection_optimizer test files             #
#                                                                              #
#------------------------------------------------------------------------------#
set_property(GLOBAL APPEND PROPERTY
    ALL_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_side_wall_detection_optimizer.cpp
    ${CMAKE_SOURCE_DIR}/src/optimizer/side_wall_detection/side_wall_detection_optimizer.cpp
    ${CMAKE_SOURCE_DIR}/src/optimizer/side_wall_detection/side_wall_detection_optimizer.hpp
)

add_executable(test_side_wall_detection_optimizer
    ${CMAKE_CURRENT_SOURCE_DIR}/test_side_wall_detection_optimizer.cpp
    ${CMAKE_SOURCE_DIR}/platforms/windows/test_main.cpp
)

target_link_libraries(test_side_wall_detection_optimizer PRIVATE
    side_wall_detection_optimizer_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_side_wall_detection_optimizer COMMAND test_side_wall_detection_optimizer)

add_custom_command(TARGET test_side_wall_detection_optimizer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:pagmo>
        $<TARGET_FILE_DIR:test_side_wall_detection_optimizer>
)
