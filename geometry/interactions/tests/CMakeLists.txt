#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for interactions test files                              #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all test and associated tested source files
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_interactions.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/interactions/interactions.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/interactions/interactions.hpp
    PARENT_SCOPE
)

add_executable(test_interactions # <=== Define your test target
    ${CMAKE_CURRENT_SOURCE_DIR}/test_interactions.cpp # <=== Add all test source files
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_interactions PRIVATE # <=== Rename to your test target
    interactions_lib # <== Link to all libraries defined by your software project needed in the executable
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_interactions COMMAND test_interactions) # <=== Rename to your test target
