#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for common simulation utilities test files               #
#                                                                              #
#------------------------------------------------------------------------------#
set_property(GLOBAL APPEND PROPERTY
    ALL_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_simulation_common.cpp
    ${CMAKE_SOURCE_DIR}/src/simulations/common/simulation_common.cpp
    ${CMAKE_SOURCE_DIR}/src/simulations/common/simulation_common.hpp
)

add_executable(test_simulation_common
    ${CMAKE_CURRENT_SOURCE_DIR}/test_simulation_common.cpp
    ${CMAKE_SOURCE_DIR}/platforms/windows/test_main.cpp
)

target_link_libraries(test_simulation_common PRIVATE
    simulation_common_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_simulation_common COMMAND test_simulation_common)
