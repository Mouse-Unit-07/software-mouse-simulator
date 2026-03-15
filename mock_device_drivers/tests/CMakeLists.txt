#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for mock_device_drivers test files                       #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all test and associated tested source files
set(ALL_TEST_SOURCES
    ${ALL_TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_mock_device_drivers.cpp
    ${CMAKE_SOURCE_DIR}/src/mock_device_drivers/mock_device_drivers.c
    ${CMAKE_SOURCE_DIR}/src/mock_device_drivers/mock_device_drivers.h
    PARENT_SCOPE
)

add_executable(test_mock_device_drivers # <=== Define your test target
    ${CMAKE_CURRENT_SOURCE_DIR}/test_mock_device_drivers.cpp # <=== Add all test source files
    ${CMAKE_SOURCE_DIR}/cmake/tests/test_main.cpp
)

target_link_libraries(test_mock_device_drivers PRIVATE # <=== Rename to your test target
    mock_device_drivers_lib # <== Link to all libraries defined by your software project needed in the executable
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_mock_device_drivers COMMAND test_mock_device_drivers) # <=== Rename to your test target
