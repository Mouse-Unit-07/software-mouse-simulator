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
    #include <math.h>
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
constexpr double FLOATING_POINT_TEST_TOLERANCE{1e-6};

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
    update_ir_2_sensor_reading(50.0);
    update_ir_3_sensor_reading(100.0);
    update_ir_4_sensor_reading(150.0);
    LONGS_EQUAL(979u, read_ir_1_sensor());
    LONGS_EQUAL(365u, read_ir_2_sensor());
    LONGS_EQUAL(155u, read_ir_3_sensor());
    LONGS_EQUAL(85u, read_ir_4_sensor());
}

TEST(MockDeviceDriversTests, NewEncoderTicksComputable)
{
    set_wheel_motor_1_direction_forward();
    set_wheel_motor_2_direction_backward();
    set_wheel_motor_1_speed(255);
    set_wheel_motor_2_speed(127);

    update_encoder_1_ticks(1.0);
    update_encoder_2_ticks(1.0);

    LONGS_EQUAL(1241, get_encoder_1_ticks());
    LONGS_EQUAL(-618, get_encoder_2_ticks());
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

    LONGS_EQUAL(0, get_encoder_1_ticks());
    LONGS_EQUAL(0, get_encoder_2_ticks());
}

TEST(MockDeviceDriversTests, NoMovementWhenPWMZero)
{
    mouse_delta delta{compute_mouse_delta(0.0, 1.0)};

    DOUBLES_EQUAL(0.0, delta.dx, FLOATING_POINT_TEST_TOLERANCE);
    DOUBLES_EQUAL(0.0, delta.dy, FLOATING_POINT_TEST_TOLERANCE);
    DOUBLES_EQUAL(0.0, delta.dtheta_rad, FLOATING_POINT_TEST_TOLERANCE);
}

TEST(MockDeviceDriversTests, StraightMotionProducesNoRotation)
{
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    mouse_delta delta{compute_mouse_delta(0.0, 1.0)};

    DOUBLES_EQUAL(0.0, delta.dy, FLOATING_POINT_TEST_TOLERANCE);
    DOUBLES_EQUAL(0.0, delta.dtheta_rad, FLOATING_POINT_TEST_TOLERANCE);

    CHECK(delta.dx > 0.0);
}

TEST(MockDeviceDriversTests, CurrentMouseAngleChangesMovementDirection)
{
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    mouse_delta delta{compute_mouse_delta(M_PI / 2.0, 1.0)};

    DOUBLES_EQUAL(0.0, delta.dx, FLOATING_POINT_TEST_TOLERANCE);
    CHECK(delta.dy > 0.0);
}

TEST(MockDeviceDriversTests, OppositeMotorsRotateInPlace)
{
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    set_wheel_motor_1_direction_backward();
    set_wheel_motor_2_direction_forward();

    mouse_delta delta{compute_mouse_delta(0.0, 1.0)};

    DOUBLES_EQUAL(0.0, delta.dx, FLOATING_POINT_TEST_TOLERANCE);
    DOUBLES_EQUAL(0.0, delta.dy, FLOATING_POINT_TEST_TOLERANCE);

    CHECK(delta.dtheta_rad != 0.0);
}

TEST(MockDeviceDriversTests, SpeedScaleHalvesDistance)
{
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    set_motor_speed_scale(1.0);
    mouse_delta normal{compute_mouse_delta(0.0, 1.0)};

    set_motor_speed_scale(0.5);
    mouse_delta scaled{compute_mouse_delta(0.0, 1.0)};

    DOUBLES_EQUAL(normal.dx * 0.5, scaled.dx, FLOATING_POINT_TEST_TOLERANCE);
}

TEST(MockDeviceDriversTests, MotorVarianceCausesTurning)
{
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    set_motor_1_variance(0.1);

    mouse_delta delta{compute_mouse_delta(0.0, 1.0)};

    CHECK(delta.dtheta_rad != 0.0);
}

TEST(MockDeviceDriversTests, SlipReducesDistance)
{
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    mouse_delta normal{compute_mouse_delta(0.0, 1.0)};

    set_motor_slip_factor(0.5);

    mouse_delta slipping{compute_mouse_delta(0.0, 1.0)};

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
    int32_t full_encoder_1_count{get_encoder_1_ticks()};
    int32_t full_encoder_2_count{get_encoder_2_ticks()};

    clear_1_encoder_ticks();
    clear_2_encoder_ticks();

    set_motor_speed_scale(0.5);
    update_encoder_1_ticks(1.0);
    update_encoder_2_ticks(1.0);
    int32_t half_encoder_1_count{get_encoder_1_ticks()};
    int32_t half_encoder_2_count{get_encoder_2_ticks()};

    LONGS_EQUAL(half_encoder_1_count, (full_encoder_1_count / 2));
    LONGS_EQUAL(half_encoder_2_count, (full_encoder_2_count / 2));
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
    int32_t full_encoder_1_count{get_encoder_1_ticks()};
    int32_t full_encoder_2_count{get_encoder_2_ticks()};

    clear_1_encoder_ticks();
    clear_2_encoder_ticks();

    set_motor_1_variance(-0.5);
    set_motor_2_variance(-0.5);
    update_encoder_1_ticks(1.0);
    update_encoder_2_ticks(1.0);
    int32_t half_encoder_1_count{get_encoder_1_ticks()};
    int32_t half_encoder_2_count{get_encoder_2_ticks()};

    LONGS_EQUAL(half_encoder_1_count, (full_encoder_1_count / 2));
    LONGS_EQUAL(half_encoder_2_count, (full_encoder_2_count / 2));
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
    int32_t full_encoder_1_count{get_encoder_1_ticks()};
    int32_t full_encoder_2_count{get_encoder_2_ticks()};

    clear_1_encoder_ticks();
    clear_2_encoder_ticks();

    set_motor_slip_factor(0.5);
    update_encoder_1_ticks(1.0);
    update_encoder_2_ticks(1.0);
    int32_t half_encoder_1_count{get_encoder_1_ticks()};
    int32_t half_encoder_2_count{get_encoder_2_ticks()};

    LONGS_EQUAL(half_encoder_1_count, (full_encoder_1_count / 2));
    LONGS_EQUAL(half_encoder_2_count, (full_encoder_2_count / 2));
}

TEST(MockDeviceDriversTests, WheelCircumferenceScaleHalvesDistance)
{
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    set_wheel_circumference_scale(1.0);
    mouse_delta normal{compute_mouse_delta(0.0, 1.0)};

    set_wheel_circumference_scale(0.5);
    mouse_delta scaled{compute_mouse_delta(0.0, 1.0)};

    DOUBLES_EQUAL(normal.dx * 0.5, scaled.dx, FLOATING_POINT_TEST_TOLERANCE);
}

TEST(MockDeviceDriversTests, WheelBaseScaleChangesDtheta)
{
    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(0);

    set_wheel_base_scale(1.0);
    mouse_delta normal{compute_mouse_delta(0.0, 1.0)};

    set_wheel_base_scale(1.1);
    mouse_delta scaled{compute_mouse_delta(0.0, 1.0)};

    CHECK(fabs(normal.dtheta_rad - scaled.dtheta_rad) > FLOATING_POINT_TEST_TOLERANCE);
}

TEST(MockDeviceDriversTests, SwappingMotorsFlipsRotation)
{
    set_wheel_motor_1_speed(100);
    set_wheel_motor_2_speed(200);

    mouse_delta a{compute_mouse_delta(0.0, 1.0)};

    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(100);

    mouse_delta b{compute_mouse_delta(0.0, 1.0)};

    DOUBLES_EQUAL(a.dtheta_rad, -b.dtheta_rad, FLOATING_POINT_TEST_TOLERANCE);
}

TEST(MockDeviceDriversTests, DistanceProportionalToPWM)
{
    set_wheel_motor_1_speed(100);
    set_wheel_motor_2_speed(100);

    mouse_delta slow{compute_mouse_delta(0.0, 1.0)};

    set_wheel_motor_1_speed(200);
    set_wheel_motor_2_speed(200);

    mouse_delta fast{compute_mouse_delta(0.0, 1.0)};

    CHECK(fast.dx > slow.dx);
}

TEST(MockDeviceDriversTests, ScaleOneReturnsSameValue)
{
    LONGS_EQUAL(100u, scale_and_clamp_ir_sensor_reading(100u, 1.0));
    LONGS_EQUAL(0u, scale_and_clamp_ir_sensor_reading(0u, 1.0));
    LONGS_EQUAL(1024u, scale_and_clamp_ir_sensor_reading(1024u, 1.0));
}

TEST(MockDeviceDriversTests, ScalingUpClampsToMax)
{
    LONGS_EQUAL(1024u, scale_and_clamp_ir_sensor_reading(800u, 2.0));
    LONGS_EQUAL(1024u, scale_and_clamp_ir_sensor_reading(1024u, 10.0));
}

TEST(MockDeviceDriversTests, ScalingDownReducesValue)
{
    LONGS_EQUAL(50u, scale_and_clamp_ir_sensor_reading(100u, 0.5));
    LONGS_EQUAL(1u, scale_and_clamp_ir_sensor_reading(2u, 0.5));
}

TEST(MockDeviceDriversTests, NegativeScaleClampsToZero)
{
    LONGS_EQUAL(0u, scale_and_clamp_ir_sensor_reading(100u, -1.0));
    LONGS_EQUAL(0u, scale_and_clamp_ir_sensor_reading(1u, -0.1));
}

TEST(MockDeviceDriversTests, RoundingBehaviorIsCorrect)
{
    /* 100 * 0.49 = 49 -> rounds to 49 */
    LONGS_EQUAL(49u, scale_and_clamp_ir_sensor_reading(100u, 0.49));

    /* 100 * 0.5 = 50 -> exact */
    LONGS_EQUAL(50u, scale_and_clamp_ir_sensor_reading(100u, 0.5));

    /* 3 * 0.5 = 1.5 -> lround -> 2 */
    LONGS_EQUAL(2u, scale_and_clamp_ir_sensor_reading(3u, 0.5));
}

TEST(MockDeviceDriversTests, ZeroInputAlwaysZero)
{
    LONGS_EQUAL(0u, scale_and_clamp_ir_sensor_reading(0u, 0.0));
    LONGS_EQUAL(0u, scale_and_clamp_ir_sensor_reading(0u, 10.0));
    LONGS_EQUAL(0u, scale_and_clamp_ir_sensor_reading(0u, -10.0));
}

TEST(MockDeviceDriversTests, LargeInputStillClamped)
{
    LONGS_EQUAL(1024u, scale_and_clamp_ir_sensor_reading(100000u, 1.0));
    LONGS_EQUAL(1024u, scale_and_clamp_ir_sensor_reading(UINT32_MAX, 1.0));
}
