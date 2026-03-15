#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for mock_device_drivers source files                     #
#                                                                              #
#------------------------------------------------------------------------------#
add_library(mock_device_drivers_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/mock_device_drivers.c
)

target_include_directories(mock_device_drivers_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${device_drivers_SOURCE_DIR}/infrared_sensor
    ${device_drivers_SOURCE_DIR}/magnetic_encoder
    ${device_drivers_SOURCE_DIR}/wheel_motor
)
