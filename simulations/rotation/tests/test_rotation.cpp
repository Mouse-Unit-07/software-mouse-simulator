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

    cfg.env_cfg.motor_speed_scale = 1.0;
    cfg.env_cfg.dt = 0.01;
    cfg.env_cfg.motor1_variance = 0.0;
    cfg.env_cfg.motor2_variance = 0.0;
    cfg.env_cfg.slip_factor = 1.0;
    cfg.env_cfg.wheel_circumference_scale = 1.0;
    cfg.env_cfg.wheel_base_scale = 1.0;
    cfg.env_cfg.rotation_angle = M_PI / 2;
    cfg.ctrl_cfg.motor_speed = 150u;
    cfg.ctrl_cfg.kp_velocity = 0;
    cfg.ctrl_cfg.kd_velocity = 0;
    cfg.ctrl_cfg.kp_angle = 0;
    cfg.ctrl_cfg.kd_angle = 0;
    cfg.ctrl_cfg.pid_scale = 1;

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
TEST(RotationTests, EncodeDecodeControlRoundTrip)
{
    ControlConfig original;
    original.motor_speed = 123u;
    original.kp_velocity = 1000;
    original.kd_velocity = -250;
    original.kp_angle = 1000;
    original.kd_angle = -250;
    original.pid_scale = 256;

    auto encoded{encode_control(original)};
    auto decoded{decode_control(encoded)};

    CHECK_EQUAL(original.motor_speed, decoded.motor_speed);
    CHECK_EQUAL(original.kp_velocity, decoded.kp_velocity);
    CHECK_EQUAL(original.kd_velocity, decoded.kd_velocity);
    CHECK_EQUAL(original.kp_angle, decoded.kp_angle);
    CHECK_EQUAL(original.kd_angle, decoded.kd_angle);
    CHECK_EQUAL(original.pid_scale, decoded.pid_scale);
}

TEST(RotationTests, EncodeControlMaintainsFieldOrder)
{
    ControlConfig cfg;
    cfg.motor_speed = 0;
    cfg.kp_velocity = 1;
    cfg.kd_velocity = 2;
    cfg.kp_angle = 3;
    cfg.kd_angle = 4;
    cfg.pid_scale = 5;

    auto v{encode_control(cfg)};

    CHECK_EQUAL(0, v.at(0));
    CHECK_EQUAL(1, v.at(1));
    CHECK_EQUAL(2, v.at(2));
    CHECK_EQUAL(3, v.at(3));
    CHECK_EQUAL(4, v.at(4));
    CHECK_EQUAL(5, v.at(5));
}

TEST(RotationTests, GetControlBoundsHasCorrectSize)
{
    auto [low, high] = get_control_bounds();

    CHECK_EQUAL(6, low.size());
    CHECK_EQUAL(6, high.size());
}

TEST(RotationTests, GetControlBoundsValuesAreCorrect)
{
    auto [low, high] = get_control_bounds();

    CHECK_EQUAL(140, low.at(0));
    CHECK_EQUAL(0, low.at(1));
    CHECK_EQUAL(0, low.at(2));
    CHECK_EQUAL(0, low.at(3));
    CHECK_EQUAL(0, low.at(4));
    CHECK_EQUAL(16, low.at(5));

    CHECK_EQUAL(255, high.at(0));
    CHECK_EQUAL(2000, high.at(1));
    CHECK_EQUAL(2000, high.at(2));
    CHECK_EQUAL(2000, high.at(3));
    CHECK_EQUAL(2000, high.at(4));
    CHECK_EQUAL(512, high.at(5));
}

TEST(RotationTests, GetControlBoundsAreDecodeSafe)
{
    auto [low, high] = get_control_bounds();

    auto low_cfg{decode_control(low)};
    auto high_cfg{decode_control(high)};

    CHECK_EQUAL(140, low_cfg.motor_speed);
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
        CHECK((e.rotation_angle >= (M_PI / 4)) && (e.rotation_angle <= M_PI / 2));
    }
}

TEST(RotationTests, SimulationProducesValidResult)
{
    Config cfg{create_no_variance_config()};

    auto r{run_simulation(cfg)};

    CHECK(r.total_time >= 0.0);
    CHECK(r.final_angle_error >= 0.0);
    CHECK(r.total_translation >= 0.0);
}

TEST(RotationTests, SimulationFailsWhenDtIsZero)
{
    Config cfg{create_no_variance_config()};
    cfg.env_cfg.dt = 0.0;

    auto r{run_simulation(cfg)};

    CHECK(r.timeout);
}

TEST(RotationTests, PositiveAndNegativeAnglesProduceSameAngleAndTranslationError)
{
    Config cfg{create_no_variance_config()};

    cfg.env_cfg.rotation_angle = M_PI / 2; 
    auto r1{run_simulation(cfg)};

    cfg.env_cfg.rotation_angle = -M_PI / 2; 
    auto r2{run_simulation(cfg)};

    CHECK_FALSE(r1.timeout);
    CHECK_FALSE(r2.timeout);
    DOUBLES_EQUAL(r1.total_translation, r2.total_translation, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(r1.final_angle_error, r2.final_angle_error, FLOAT_TOLERANCE);
}

TEST(RotationTests, LargerAngleTakesMoreTime)
{
    Config cfg{create_no_variance_config()};

    cfg.env_cfg.rotation_angle = M_PI / 4;
    auto small{run_simulation(cfg)};

    cfg.env_cfg.rotation_angle = M_PI / 2;
    auto large{run_simulation(cfg)};

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
    cfg.ctrl_cfg.motor_speed = 255u;
    cfg.env_cfg.motor1_variance = -1;
    cfg.env_cfg.rotation_angle = M_PI;

    auto r{run_simulation(cfg)};

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
    cfg.ctrl_cfg.motor_speed = 100;
    cfg.env_cfg.dt = 0.001;
    cfg.env_cfg.rotation_angle = M_PI;

    auto r{run_simulation(cfg)};

    /* 1% of a circle, or 3.6 degrees */
    constexpr double ROTATION_TOLERANCE{(2 * M_PI) * 0.01};

    CHECK_FALSE(r.timeout);
    DOUBLES_EQUAL(0.0, r.final_angle_error, ROTATION_TOLERANCE);
    DOUBLES_EQUAL(0.0, r.total_translation, 0.01);
}

TEST(RotationTests, DerivativeTermAffectsStability)
{
    Config cfg{create_no_variance_config()};
    cfg.env_cfg.motor1_variance = -0.2;
    cfg.ctrl_cfg.kp_velocity = 2000;
    cfg.ctrl_cfg.pid_scale = 256;

    Config no_d{cfg};
    Config with_d{cfg};
    with_d.ctrl_cfg.kd_velocity = 1000;

    auto r1{run_simulation(no_d)};
    auto r2{run_simulation(with_d)};

    CHECK((r1.final_angle_error != r2.final_angle_error)
          || (r1.total_translation != r2.total_translation));
}

TEST(RotationTests, PidShiftAffectsControlStrength)
{
    Config cfg{create_no_variance_config()};
    cfg.env_cfg.motor1_variance = -0.2;
    cfg.ctrl_cfg.kp_velocity = 2000;
    cfg.ctrl_cfg.kd_velocity = 1000;

    Config strong{cfg};
    strong.ctrl_cfg.pid_scale = 4;
    Config weak{cfg};
    weak.ctrl_cfg.pid_scale = 256;

    auto r1{run_simulation(strong)};
    auto r2{run_simulation(weak)};

    CHECK((r1.total_time != r2.total_time) || (r1.final_angle_error != r2.final_angle_error));
}

IGNORE_TEST(RotationTests, VisualizationDoesNotAffectResults)
{
    Config cfg{create_no_variance_config()};

    disable_visualization();
    auto r1{run_simulation(cfg)};

    enable_visualization("visualization-does-not-affect-results");
    auto r2{run_simulation(cfg)};

    CHECK(are_results_equivalent(r1, r2));
}
