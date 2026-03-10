#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for physics test files                                   #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all test and associated tested source files
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_physics.cpp
    ${CMAKE_SOURCE_DIR}/src/physics/physics.cpp
    ${CMAKE_SOURCE_DIR}/src/physics/physics.hpp
    PARENT_SCOPE
)

add_executable(test_physics # <=== Define your test target
    ${CMAKE_CURRENT_SOURCE_DIR}/test_physics.cpp # <=== Add all test source files
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_physics PRIVATE # <=== Rename to your test target
    physics_lib # <== Link to all libraries defined by your software project needed in the executable
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_physics COMMAND test_physics) # <=== Rename to your test target
