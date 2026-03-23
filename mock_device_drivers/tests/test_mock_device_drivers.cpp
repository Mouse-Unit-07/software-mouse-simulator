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
    update_ir_1_sensor_reading(0.0);
    update_ir_2_sensor_reading(500.0);
    update_ir_3_sensor_reading(1000.0);
    update_ir_4_sensor_reading(1500.0);
    CHECK(read_ir_1_sensor() == 1024);
    CHECK(read_ir_2_sensor() == 495);
    CHECK(read_ir_3_sensor() == 230);
    CHECK(read_ir_4_sensor() == 107);
}

TEST(MockDeviceDriversTests, NewEncoderTicksComputable)
{
    set_wheel_motor_1_direction_forward();
    set_wheel_motor_2_direction_backward();
    set_wheel_motor_1_speed(255);
    set_wheel_motor_2_speed(127);

    update_encoder_1_ticks(1.0);
    update_encoder_2_ticks(1.0);

    CHECK(get_encoder_1_ticks() == 1241);
    CHECK(get_encoder_2_ticks() == -618);
}

TEST(MockDeviceDriversTests, EncoderTicksClearable)
{
    set_wheel_motor_1_direction_forward();
    set_wheel_motor_2_direction_backward();
    set_wheel_motor_1_speed(255);
    set_wheel_motor_2_speed(255);
    update_encoder_1_ticks(1.0);
    update_encoder_2_ticks(1.0);

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

TEST(MockDeviceDriversTests, SpeedScaleHalvesEncoderCount)
{
    set_wheel_motor_1_direction_forward();
    set_wheel_motor_2_direction_backward();
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    set_motor_speed_scale(1.0);
    update_encoder_1_ticks(1.0);
    update_encoder_2_ticks(1.0);
    int32_t full_encoder_1_count = get_encoder_1_ticks();
    int32_t full_encoder_2_count = get_encoder_2_ticks();

    clear_1_encoder_ticks();
    clear_2_encoder_ticks();

    set_motor_speed_scale(0.5);
    update_encoder_1_ticks(1.0);
    update_encoder_2_ticks(1.0);
    int32_t half_encoder_1_count = get_encoder_1_ticks();
    int32_t half_encoder_2_count = get_encoder_2_ticks();

    CHECK((full_encoder_1_count / 2) == half_encoder_1_count);
    CHECK((full_encoder_2_count / 2) == half_encoder_2_count);
}

TEST(MockDeviceDriversTests, MotorVarianceHalvesEncoderCount)
{
    set_wheel_motor_1_direction_forward();
    set_wheel_motor_2_direction_backward();
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    set_motor_1_variance(0);
    set_motor_2_variance(0);
    update_encoder_1_ticks(1.0);
    update_encoder_2_ticks(1.0);
    int32_t full_encoder_1_count = get_encoder_1_ticks();
    int32_t full_encoder_2_count = get_encoder_2_ticks();

    clear_1_encoder_ticks();
    clear_2_encoder_ticks();

    set_motor_1_variance(-0.5);
    set_motor_2_variance(-0.5);
    update_encoder_1_ticks(1.0);
    update_encoder_2_ticks(1.0);
    int32_t half_encoder_1_count = get_encoder_1_ticks();
    int32_t half_encoder_2_count = get_encoder_2_ticks();

    CHECK((full_encoder_1_count / 2) == half_encoder_1_count);
    CHECK((full_encoder_2_count / 2) == half_encoder_2_count);
}

TEST(MockDeviceDriversTests, SlipHalvesEncoderCount)
{
    set_wheel_motor_1_direction_forward();
    set_wheel_motor_2_direction_backward();
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    set_motor_slip_factor(1.0);
    update_encoder_1_ticks(1.0);
    update_encoder_2_ticks(1.0);
    int32_t full_encoder_1_count = get_encoder_1_ticks();
    int32_t full_encoder_2_count = get_encoder_2_ticks();

    clear_1_encoder_ticks();
    clear_2_encoder_ticks();

    set_motor_slip_factor(0.5);
    update_encoder_1_ticks(1.0);
    update_encoder_2_ticks(1.0);
    int32_t half_encoder_1_count = get_encoder_1_ticks();
    int32_t half_encoder_2_count = get_encoder_2_ticks();

    CHECK((full_encoder_1_count / 2) == half_encoder_1_count);
    CHECK((full_encoder_2_count / 2) == half_encoder_2_count);
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
