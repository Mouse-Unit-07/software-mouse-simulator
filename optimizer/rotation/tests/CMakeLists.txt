#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for rotation_optimizer test files                        #
#                                                                              #
#------------------------------------------------------------------------------#
set_property(GLOBAL APPEND PROPERTY
    ALL_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_rotation_optimizer.cpp
    ${CMAKE_SOURCE_DIR}/src/optimizer/rotation/rotation_optimizer.cpp
    ${CMAKE_SOURCE_DIR}/src/optimizer/rotation/rotation_optimizer.hpp
)

add_executable(test_rotation_optimizer
    ${CMAKE_CURRENT_SOURCE_DIR}/test_rotation_optimizer.cpp
    ${CMAKE_SOURCE_DIR}/platforms/windows/test_main.cpp
)

target_link_libraries(test_rotation_optimizer PRIVATE
    rotation_optimizer_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_rotation_optimizer COMMAND test_rotation_optimizer)

add_custom_command(TARGET test_rotation_optimizer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:pagmo>
        $<TARGET_FILE_DIR:test_rotation_optimizer>
)
