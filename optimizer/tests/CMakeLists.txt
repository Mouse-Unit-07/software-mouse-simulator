#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for optimizer test files                                 #
#                                                                              #
#------------------------------------------------------------------------------#
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_optimizer.cpp
    ${CMAKE_SOURCE_DIR}/src/optimizer/optimizer.cpp
    ${CMAKE_SOURCE_DIR}/src/optimizer/optimizer.hpp
    PARENT_SCOPE
)

add_executable(test_optimizer
    ${CMAKE_CURRENT_SOURCE_DIR}/test_optimizer.cpp
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_optimizer PRIVATE
    optimizer_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_optimizer COMMAND test_optimizer)
