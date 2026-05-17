#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for mock_device_drivers test files                       #
#                                                                              #
#------------------------------------------------------------------------------#
set_property(GLOBAL APPEND PROPERTY
    ALL_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_mock_device_drivers.cpp
    ${CMAKE_SOURCE_DIR}/src/mock_device_drivers/mock_device_drivers.c
    ${CMAKE_SOURCE_DIR}/src/mock_device_drivers/mock_device_drivers.h
)

add_executable(test_mock_device_drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/test_mock_device_drivers.cpp
    ${CMAKE_SOURCE_DIR}/platforms/windows/test_main.cpp
)

target_link_libraries(test_mock_device_drivers PRIVATE
    mock_device_drivers_lib
    CppUTest::CppUTest
    CppUTestExt
)

add_test(NAME test_mock_device_drivers COMMAND test_mock_device_drivers)
