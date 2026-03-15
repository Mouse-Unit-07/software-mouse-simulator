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
    #include <math.h>
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
constexpr double FLOATING_POINT_TEST_TOLERANCE {1e-6};

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

TEST(MockDeviceDriversTests, EncoderTicksSettableAndAccessible)
{
    int32_t encoder_1_ticks = 1000;
    int32_t encoder_2_ticks = -1000;
    set_encoder_1_ticks(encoder_1_ticks);
    set_encoder_2_ticks(encoder_2_ticks);

    CHECK(get_encoder_1_ticks() == encoder_1_ticks);
    CHECK(get_encoder_2_ticks() == encoder_2_ticks);
}

TEST(MockDeviceDriversTests, EncoderTicksClearable)
{
    set_encoder_1_ticks(1000);
    set_encoder_2_ticks(-1000);
    clear_1_encoder_ticks();
    clear_2_encoder_ticks();

    CHECK(get_encoder_1_ticks() == 0);
    CHECK(get_encoder_2_ticks() == 0);
}

TEST(MockDeviceDriversTests, NoMovementWhenPWMZero)
{
    mouse_delta delta = compute_mouse_delta(0.0, 1.0);

    DOUBLES_EQUAL(0.0, delta.dx, FLOATING_POINT_TEST_TOLERANCE);
    DOUBLES_EQUAL(0.0, delta.dy, FLOATING_POINT_TEST_TOLERANCE);
    DOUBLES_EQUAL(0.0, delta.dtheta_rad, FLOATING_POINT_TEST_TOLERANCE);
}

TEST(MockDeviceDriversTests, StraightMotionProducesNoRotation)
{
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    mouse_delta delta = compute_mouse_delta(0.0, 1.0);

    DOUBLES_EQUAL(0.0, delta.dy, FLOATING_POINT_TEST_TOLERANCE);
    DOUBLES_EQUAL(0.0, delta.dtheta_rad, FLOATING_POINT_TEST_TOLERANCE);

    CHECK(delta.dx > 0.0);
}

TEST(MockDeviceDriversTests, CurrentMouseAngleChangesMovementDirection)
{
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    mouse_delta delta = compute_mouse_delta(M_PI / 2.0, 1.0);

    DOUBLES_EQUAL(0.0, delta.dx, FLOATING_POINT_TEST_TOLERANCE);
    CHECK(delta.dy > 0.0);
}

TEST(MockDeviceDriversTests, OppositeMotorsRotateInPlace)
{
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    set_wheel_motor_1_direction_backward();
    set_wheel_motor_2_direction_forward();

    mouse_delta delta = compute_mouse_delta(0.0, 1.0);

    DOUBLES_EQUAL(0.0, delta.dx, FLOATING_POINT_TEST_TOLERANCE);
    DOUBLES_EQUAL(0.0, delta.dy, FLOATING_POINT_TEST_TOLERANCE);

    CHECK(delta.dtheta_rad != 0.0);
}

TEST(MockDeviceDriversTests, SpeedScaleHalvesDistance)
{
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    set_motor_speed_scale(1.0);
    mouse_delta normal = compute_mouse_delta(0.0, 1.0);

    set_motor_speed_scale(0.5);
    mouse_delta scaled = compute_mouse_delta(0.0, 1.0);

    DOUBLES_EQUAL(normal.dx * 0.5, scaled.dx, FLOATING_POINT_TEST_TOLERANCE);
}

TEST(MockDeviceDriversTests, MotorVarianceCausesTurning)
{
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    set_motor_1_variance(0.1);

    mouse_delta delta = compute_mouse_delta(0.0, 1.0);

    CHECK(delta.dtheta_rad != 0.0);
}

TEST(MockDeviceDriversTests, SlipReducesDistance)
{
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    mouse_delta normal = compute_mouse_delta(0.0, 1.0);

    set_motor_slip_factor(0.5);

    mouse_delta slipping = compute_mouse_delta(0.0, 1.0);

    CHECK(slipping.dx < normal.dx);
}

TEST(MockDeviceDriversTests, WheelCircumferenceScaleHalvesDistance)
{
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    set_wheel_circumference_scale(1.0);
    mouse_delta normal = compute_mouse_delta(0.0, 1.0);

    set_wheel_circumference_scale(0.5);
    mouse_delta scaled = compute_mouse_delta(0.0, 1.0);

    DOUBLES_EQUAL(normal.dx * 0.5, scaled.dx, FLOATING_POINT_TEST_TOLERANCE);
}

TEST(MockDeviceDriversTests, WheelBaseScaleChangesDtheta)
{
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(0);

    set_wheel_base_scale(1.0);
    mouse_delta normal = compute_mouse_delta(0.0, 1.0);

    set_wheel_base_scale(1.1);
    mouse_delta scaled = compute_mouse_delta(0.0, 1.0);

    CHECK(fabs(normal.dtheta_rad - scaled.dtheta_rad) > FLOATING_POINT_TEST_TOLERANCE);
}

TEST(MockDeviceDriversTests, SwappingMotorsFlipsRotation)
{
    set_wheel_motor_1_speed(100);
    set_wheel_motor_2_speed(200);

    mouse_delta a = compute_mouse_delta(0.0, 1.0);

    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(100);

    mouse_delta b = compute_mouse_delta(0.0, 1.0);

    DOUBLES_EQUAL(a.dtheta_rad, -b.dtheta_rad, FLOATING_POINT_TEST_TOLERANCE);
}

TEST(MockDeviceDriversTests, DistanceProportionalToPWM)
{
    set_wheel_motor_1_speed(100);
    set_wheel_motor_2_speed(100);

    mouse_delta slow = compute_mouse_delta(0.0, 1.0);

    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    mouse_delta fast = compute_mouse_delta(0.0, 1.0);

    CHECK(fast.dx > slow.dx);
}
