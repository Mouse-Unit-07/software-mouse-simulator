/*================================ FILE INFO =================================*/
/* Filename           : test_mock_device_drivers.cpp                          */
/*                                                                            */
/* Test implementation for mock_device_drivers.cpp                            */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
extern "C"
{
    #include <stdint.h>
    #include "infrared_sensor.h"
    #include "magnetic_encoder.h"
    #include "wheel_motor.h"
    #include "mock_device_drivers.h"
}

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/


/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(MockDeviceDriversTests)
{
    void setup() override
    {
        reset_mock_device_drivers();
    }

    void teardown() override
    {
        reset_mock_device_drivers();
    }
};

/*============================================================================*/
/*                                    Tests                                   */
/*============================================================================*/
TEST(MockDeviceDriversTests, DesiredIrSensorValuesSettableAndGettable)
{
    set_ir_1_sensor_reading(compute_ir_sensor_reading_from_distance_mm(0.0));
    set_ir_2_sensor_reading(compute_ir_sensor_reading_from_distance_mm(500.0));
    set_ir_3_sensor_reading(compute_ir_sensor_reading_from_distance_mm(1000.0));
    set_ir_4_sensor_reading(compute_ir_sensor_reading_from_distance_mm(1500.0));
    CHECK(read_ir_1_sensor() == 1024);
    CHECK(read_ir_2_sensor() == 495);
    CHECK(read_ir_3_sensor() == 230);
    CHECK(read_ir_4_sensor() == 107);
}

TEST(MockDeviceDriversTests, NewEncoderTicksComputable)
{
    uint8_t motor_1_speed = 255;
    uint8_t motor_2_speed = 127;
    set_wheel_motor_1_direction_forward();
    set_wheel_motor_2_direction_backward();
    set_wheel_motor_1_speed(motor_1_speed);
    set_wheel_motor_2_speed(motor_2_speed);

    CHECK(compute_new_encoder_1_ticks(1.0) == 1241);
    CHECK(compute_new_encoder_2_ticks(1.0) == -618);
}
