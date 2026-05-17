#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for optimizer_common test files                          #
#                                                                              #
#------------------------------------------------------------------------------#
set_property(GLOBAL APPEND PROPERTY
    ALL_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_optimizer_common.cpp
    ${CMAKE_SOURCE_DIR}/src/optimizer/common/optimizer_common.cpp
    ${CMAKE_SOURCE_DIR}/src/optimizer/common/optimizer_common.hpp
)

add_executable(test_optimizer_common
    ${CMAKE_CURRENT_SOURCE_DIR}/test_optimizer_common.cpp
    ${CMAKE_SOURCE_DIR}/platforms/windows/test_main.cpp
)

target_link_libraries(test_optimizer_common PRIVATE
    optimizer_common_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_optimizer_common COMMAND test_optimizer_common)

add_custom_command(TARGET test_optimizer_common POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:pagmo>
        $<TARGET_FILE_DIR:test_optimizer_common>
)
