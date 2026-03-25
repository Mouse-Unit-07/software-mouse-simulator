#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for common simulation utilities test files               #
#                                                                              #
#------------------------------------------------------------------------------#
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_simulation_common.cpp
    ${CMAKE_SOURCE_DIR}/src/simulations/common/simulation_common.cpp
    ${CMAKE_SOURCE_DIR}/src/simulations/common/simulation_common.hpp
    PARENT_SCOPE
)

add_executable(test_simulation_common
    ${CMAKE_CURRENT_SOURCE_DIR}/test_simulation_common.cpp
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_simulation_common PRIVATE
    simulation_common_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_simulation_common COMMAND test_simulation_common)
