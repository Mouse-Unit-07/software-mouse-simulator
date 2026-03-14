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
    DOUBLES_EQUAL(1024.0, read_ir_1_sensor(), 1e-6);
    DOUBLES_EQUAL(495, read_ir_2_sensor(), 1e-6);
    DOUBLES_EQUAL(230, read_ir_3_sensor(), 1e-6);
    DOUBLES_EQUAL(107, read_ir_4_sensor(), 1e-6);
}
