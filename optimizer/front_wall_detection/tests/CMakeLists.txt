#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for front_wall_detection_optimizer test files            #
#                                                                              #
#------------------------------------------------------------------------------#
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_front_wall_detection_optimizer.cpp
    ${CMAKE_SOURCE_DIR}/src/optimizer/front_wall_detection/front_wall_detection_optimizer.cpp
    ${CMAKE_SOURCE_DIR}/src/optimizer/front_wall_detection/front_wall_detection_optimizer.hpp
    PARENT_SCOPE
)

add_executable(test_front_wall_detection_optimizer
    ${CMAKE_CURRENT_SOURCE_DIR}/test_front_wall_detection_optimizer.cpp
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_front_wall_detection_optimizer PRIVATE
    front_wall_detection_optimizer_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_front_wall_detection_optimizer COMMAND test_front_wall_detection_optimizer)

add_custom_command(TARGET test_front_wall_detection_optimizer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:pagmo>
        $<TARGET_FILE_DIR:test_front_wall_detection_optimizer>
)
