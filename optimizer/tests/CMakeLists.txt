#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for optimizer test files                                 #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all test and associated tested source files
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_optimizer.cpp
    ${CMAKE_SOURCE_DIR}/src/optimizer/optimizer.cpp
    ${CMAKE_SOURCE_DIR}/src/optimizer/optimizer.hpp
    PARENT_SCOPE
)

add_executable(test_optimizer # <=== Define your test target
    ${CMAKE_CURRENT_SOURCE_DIR}/test_optimizer.cpp # <=== Add all test source files
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_optimizer PRIVATE # <=== Rename to your test target
    optimizer_lib # <== Link to all libraries defined by your software project needed in the executable
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_optimizer COMMAND test_optimizer) # <=== Rename to your test target
