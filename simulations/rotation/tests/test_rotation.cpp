/*================================ FILE INFO =================================*/
/* Filename           : test_rotation.cpp                                     */
/*                                                                            */
/* Test implementation for rotation.cpp                                       */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
#include "simulation_common.hpp"
#include "rotation.hpp"

using namespace rotation;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
constexpr double FLOAT_TOLERANCE{1e-6};

Config create_no_variance_config(void)
{
    Config cfg;

    cfg.motor_speed = 150u;
    cfg.motor_speed_scale = 1.0;
    cfg.dt = 0.01;
    cfg.motor1_variance = 0.0;
    cfg.motor2_variance = 0.0;
    cfg.slip_factor = 1.0;
    cfg.wheel_circumference_scale = 1.0;
    cfg.wheel_base_scale = 1.0;
    cfg.kp = 0;
    cfg.kd = 0;
    cfg.pid_shift = 0;

    return cfg;
}

Config create_config_custom_pid_and_speed(int kp, int kd, int shift, uint8_t motor_speed = 0)
{
    Config cfg{create_no_variance_config()};

    cfg.kp = kp;
    cfg.kd = kd;
    cfg.pid_shift = shift;
    cfg.motor_speed = motor_speed;

    return cfg;
}

bool are_results_equivalent(const Result& r1, const Result& r2)
{
    if (std::abs(r1.total_time - r2.total_time) >= FLOAT_TOLERANCE) {
        return false;
    }
    if (std::abs(r1.final_angle_error - r2.final_angle_error) >= FLOAT_TOLERANCE) {
        return false;
    }
    if (std::abs(r1.total_translation - r2.total_translation) >= FLOAT_TOLERANCE) {
        return false;
    }

    if (r1.collision != r2.collision) {
        return false;
    }
    if (r1.timeout != r2.timeout) {
        return false;
    }

    return true;
}

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(RotationTests)
{
    void setup() override
    {
        disable_visualization();
    }

    void teardown() override
    {
        disable_visualization();
    }
};

/*============================================================================*/
/*                                    Tests                                   */
/*============================================================================*/
TEST(RotationTests, SimulationProducesValidResult)
{
    Config cfg{create_no_variance_config()};

    auto r{run_simulation(cfg, M_PI / 2)};

    CHECK(r.total_time >= 0.0);
    CHECK(r.final_angle_error >= 0.0);
    CHECK(r.total_translation >= 0.0);
}

TEST(RotationTests, SimulationFailsWhenDtIsZero)
{
    Config cfg{create_no_variance_config()};
    cfg.dt = 0.0;

    auto r{run_simulation(cfg, M_PI / 2)};

    CHECK(r.timeout);
}

TEST(RotationTests, PositiveAndNegativeAnglesProduceSameAngleAndTranslationError)
{
    Config cfg{create_no_variance_config()};

    auto r1{run_simulation(cfg, M_PI / 2)};
    auto r2{run_simulation(cfg, -M_PI / 2)};

    CHECK_FALSE(r1.timeout);
    CHECK_FALSE(r2.timeout);
    CHECK(r1.total_translation == r2.total_translation);
    CHECK(r1.final_angle_error == r2.final_angle_error);
}

TEST(RotationTests, LargerAngleTakesMoreTime)
{
    Config cfg{create_no_variance_config()};

    auto small{run_simulation(cfg, M_PI / 4)};
    auto large{run_simulation(cfg, M_PI / 2)};

    CHECK(large.total_time >= small.total_time);
}

TEST(RotationTests, SimulationCanDetectCollision)
{
    std::vector<std::string> ascii{
        "+-+-+",
        "|S  |",
        "+-+-+"
    };

    maze::Maze maze{maze::build_maze_from_ascii(ascii, 0.0)};

    Config cfg{create_no_variance_config()};
    cfg.motor_speed = 255u;
    cfg.motor1_variance = -1;

    auto r{run_simulation(cfg, M_PI)};

    CHECK(r.collision);
}

TEST(RotationTests, NoTranslationAndAngleErrorForPerfectTestVariables)
{
    std::vector<std::string> ascii{
        "+-+",
        "|S|",
        "+-+"
    };
    maze::Maze maze{maze::build_maze_from_ascii(ascii, 0.0)};

    /* slow movement, tiny dt, and no motor variances */
    Config cfg{create_no_variance_config()};
    cfg.motor_speed = 100;
    cfg.dt = 0.001;

    auto r{run_simulation(cfg, M_PI)};

    /* 1% of a circle, or 3.6 degrees */
    constexpr double ROTATION_TOLERANCE{(2 * M_PI) * 0.01};

    CHECK_FALSE(r.timeout);
    DOUBLES_EQUAL(0.0, r.final_angle_error, ROTATION_TOLERANCE);
    DOUBLES_EQUAL(0.0, r.total_translation, 0.01);
}

TEST(RotationTests, DerivativeTermAffectsStability)
{
    Config cfg{create_no_variance_config()};
    cfg.motor1_variance = -0.2;
    cfg.kp = 2000;
    cfg.pid_shift = 8;

    Config no_d{cfg};
    Config with_d{cfg};
    with_d.kd = 1000;

    auto r1{run_simulation(no_d, M_PI / 2)};
    auto r2{run_simulation(with_d, M_PI / 2)};

    CHECK((r1.final_angle_error != r2.final_angle_error)
          || (r1.total_translation != r2.total_translation));
}

TEST(RotationTests, PDImprovesAccuracyOverNoControl)
{
    Config cfg{create_no_variance_config()};
    cfg.motor1_variance = -0.2;
    cfg.pid_shift = 8;

    Config no_control{cfg};
    Config pd_control{cfg};
    pd_control.kp = 2000;
    pd_control.kd = 1000;

    auto r1{run_simulation(no_control, M_PI / 2)};
    auto r2{run_simulation(pd_control, M_PI / 2)};

    CHECK(r2.final_angle_error <= r1.final_angle_error);
}

TEST(RotationTests, PidShiftAffectsControlStrength)
{
    Config cfg{create_no_variance_config()};
    cfg.motor1_variance = -0.2;
    cfg.kp = 2000;
    cfg.kd = 1000;

    Config strong{cfg};
    strong.pid_shift = 2;
    Config weak{cfg};
    weak.pid_shift = 8;

    auto r1{run_simulation(strong, M_PI / 2)};
    auto r2{run_simulation(weak, M_PI / 2)};

    CHECK((r1.total_time != r2.total_time) || (r1.final_angle_error != r2.final_angle_error));
}

IGNORE_TEST(RotationTests, VisualizationDoesNotAffectResults)
{
    Config cfg{create_no_variance_config()};

    disable_visualization();
    auto r1{run_simulation(cfg, M_PI / 2)};

    enable_visualization();
    auto r2{run_simulation(cfg, M_PI / 2)};

    CHECK(are_results_equivalent(r1, r2));
}

TEST(RotationTests, EncodeDecodeControlRoundTrip)
{
    ControlConfig original;
    original.motor_speed = 123u;
    original.kp = 1000;
    original.kd = -250;
    original.pid_shift = 6;

    auto encoded{encode_control(original)};
    auto decoded{decode_control(encoded)};

    CHECK_EQUAL(original.motor_speed, decoded.motor_speed);
    CHECK_EQUAL(original.kp, decoded.kp);
    CHECK_EQUAL(original.kd, decoded.kd);
    CHECK_EQUAL(original.pid_shift, decoded.pid_shift);
}

TEST(RotationTests, EncodeControlMaintainsFieldOrder)
{
    ControlConfig cfg;
    cfg.motor_speed = 1;
    cfg.kp = 2;
    cfg.kd = 3;
    cfg.pid_shift = 4;

    auto v{encode_control(cfg)};

    CHECK_EQUAL(1, v.at(0));
    CHECK_EQUAL(2, v.at(1));
    CHECK_EQUAL(3, v.at(2));
    CHECK_EQUAL(4, v.at(3));
}

TEST(RotationTests, GetControlBoundsHasCorrectSize)
{
    auto [low, high] = get_control_bounds();

    CHECK_EQUAL(4, low.size());
    CHECK_EQUAL(4, high.size());
}

TEST(RotationTests, GetControlBoundsValuesAreCorrect)
{
    auto [low, high] = get_control_bounds();

    CHECK_EQUAL(100, low.at(0));
    CHECK_EQUAL(0, low.at(1));
    CHECK_EQUAL(0, low.at(2));
    CHECK_EQUAL(4, low.at(3));

    CHECK_EQUAL(255, high.at(0));
    CHECK_EQUAL(2000, high.at(1));
    CHECK_EQUAL(2000, high.at(2));
    CHECK_EQUAL(8, high.at(3));
}

TEST(RotationTests, GetControlBoundsAreDecodeSafe)
{
    auto [low, high] = get_control_bounds();

    auto low_cfg{decode_control(low)};
    auto high_cfg{decode_control(high)};

    CHECK_EQUAL(100, low_cfg.motor_speed);
    CHECK_EQUAL(255, high_cfg.motor_speed);
}

TEST(RotationTests, RandomEnvironmentValuesWithinExpectedRanges)
{
    for (int i{0}; i < 100; i++) {
        auto e{generate_random_environment()};

        CHECK((e.dt >= 0.01) && (e.dt <= 0.1));
        CHECK((e.motor_speed_scale >= 0.9) && (e.motor_speed_scale <= 1.1));
        CHECK((e.motor1_variance >= -0.2) && (e.motor1_variance <= 0.2));
        CHECK((e.motor2_variance >= -0.2) && (e.motor2_variance <= 0.2));
        CHECK((e.slip_factor >= 0.9) && (e.slip_factor <= 1.1));
        CHECK((e.wheel_circumference_scale >= 0.9) && (e.wheel_circumference_scale <= 1.1));
        CHECK((e.wheel_base_scale >= 0.9) && (e.wheel_base_scale <= 1.1));
    }
}

TEST(RotationTests, MergeControlAndEnvironmentMapsFieldsCorrectly)
{
    ControlConfig c;
    c.motor_speed = 50;
    c.kp = 100;
    c.kd = 200;
    c.pid_shift = 3;

    EnvironmentConfig e;
    e.dt = 0.02;
    e.motor_speed_scale = 1.1;
    e.motor1_variance = -0.1;
    e.motor2_variance = 0.2;
    e.slip_factor = 1.0;
    e.wheel_circumference_scale = 0.95;
    e.wheel_base_scale = 1.05;

    auto cfg{merge_control_and_environment(c, e)};

    CHECK_EQUAL(50, cfg.motor_speed);
    CHECK_EQUAL(100, cfg.kp);
    CHECK_EQUAL(200, cfg.kd);
    CHECK_EQUAL(3, cfg.pid_shift);

    DOUBLES_EQUAL(0.02, cfg.dt, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(1.1, cfg.motor_speed_scale, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(-0.1, cfg.motor1_variance, FLOAT_TOLERANCE);
}
