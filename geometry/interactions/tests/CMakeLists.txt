#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for interactions test files                              #
#                                                                              #
#------------------------------------------------------------------------------#
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_interactions.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/interactions/interactions.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/interactions/interactions.hpp
    PARENT_SCOPE
)

add_executable(test_interactions
    ${CMAKE_CURRENT_SOURCE_DIR}/test_interactions.cpp
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_interactions PRIVATE
    interactions_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_interactions COMMAND test_interactions)
