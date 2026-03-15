#--------------------------------- FILE INFO ----------------------------------#
# Filename           : CMakeLists.txt                                          #
#                                                                              #
# CMakeLists.txt file for mock_device_drivers source files                     #
#                                                                              #
#------------------------------------------------------------------------------#
# \/=== Add all .c files to your build target
add_library(mock_device_drivers_lib STATIC # <== Define this library's build target
    ${CMAKE_CURRENT_LIST_DIR}/mock_device_drivers.c
)

# \/=== Add all directories w/ header files
target_include_directories(mock_device_drivers_lib PUBLIC # <=== Rename to your build target
    ${CMAKE_CURRENT_LIST_DIR}
    ${device_drivers_SOURCE_DIR}/infrared_sensor
    ${device_drivers_SOURCE_DIR}/magnetic_encoder
    ${device_drivers_SOURCE_DIR}/wheel_motor
)
