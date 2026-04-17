/*================================ FILE INFO =================================*/
/* Filename           : test_move_forward.cpp                                 */
/*                                                                            */
/* Test implementation for move_forward.cpp                                   */
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
#include "move_forward.hpp"

using namespace move_forward;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
constexpr double FLOAT_TOLERANCE{1e-6};

Config create_no_variance_config(void)
{
    Config cfg{};

    cfg.env_cfg.dt = 0.001;
    cfg.env_cfg.motor_speed_scale = 1.0;
    cfg.env_cfg.motor1_variance = 0.0;
    cfg.env_cfg.motor2_variance = 0.0;
    cfg.env_cfg.slip_factor = 1.0;
    cfg.env_cfg.wheel_circumference_scale = 1.0;
    cfg.env_cfg.wheel_base_scale = 1.0;
    cfg.env_cfg.maze_size_scale = 1.0;
    cfg.env_cfg.ir_reading_scale = 1.0;
    cfg.env_cfg.mouse_angle = 0.0;
    cfg.env_cfg.horizontal_position_variance = 0.0;
    cfg.env_cfg.vertical_position_variance = 0.0;

    cfg.ctrl_cfg.single_wall_target = 407u;
    cfg.ctrl_cfg.motor_speed = 120u;
    cfg.ctrl_cfg.kp_velocity = 0;
    cfg.ctrl_cfg.kd_velocity = 0;
    cfg.ctrl_cfg.kp_angle = 0;
    cfg.ctrl_cfg.kd_angle = 0;
    cfg.ctrl_cfg.pid_scale = 1;
    cfg.ctrl_cfg.kp_ir = 0;
    cfg.ctrl_cfg.kd_ir = 0;

    return cfg;
}

bool was_there_collision_or_timeout(const Result& result)
{
    if (result.collision || result.timeout) {
        return true;
    }

    return false;
}

bool are_results_equivalent(const Result& r1, const Result& r2)
{
    if (std::abs(r1.total_time - r2.total_time) >= FLOAT_TOLERANCE) {
        return false;
    }
    if (std::abs(r1.total_angle_error - r2.total_angle_error) >= FLOAT_TOLERANCE) {
        return false;
    }
    if (std::abs(r1.total_horizontal_translation - r2.total_horizontal_translation)
        >= FLOAT_TOLERANCE) {
        return false;
    }
    if (std::abs(r1.final_vertical_translation - r2.final_vertical_translation)
        >= FLOAT_TOLERANCE) {
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

bool are_results_equivalent_for_wall_mode(const Config& cfg1, const Config& cfg2, WallMode mode)
{
    auto r1{run_simulation(cfg1, mode)};
    auto r2{run_simulation(cfg2, mode)};

    if (!are_results_equivalent(r1, r2)) {
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
TEST_GROUP(MoveForwardTests)
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
TEST(MoveForwardTests, EncodeDecodeControlRoundTrip)
{
    ControlConfig original;
    original.single_wall_target = 456u;
    original.motor_speed = 123u;
    original.kp_velocity = 1000;
    original.kd_velocity = -250;
    original.kp_angle = 1000;
    original.kd_angle = -250;
    original.pid_scale = 6;
    original.kp_ir = 100;
    original.kd_ir = 200;

    auto encoded{encode_control(original)};
    auto decoded{decode_control(encoded)};

    CHECK_EQUAL(original.single_wall_target, decoded.single_wall_target);
    CHECK_EQUAL(original.motor_speed, decoded.motor_speed);
    CHECK_EQUAL(original.kp_velocity, decoded.kp_velocity);
    CHECK_EQUAL(original.kd_velocity, decoded.kd_velocity);
    CHECK_EQUAL(original.kp_angle, decoded.kp_angle);
    CHECK_EQUAL(original.kd_angle, decoded.kd_angle);
    CHECK_EQUAL(original.pid_scale, decoded.pid_scale);
    CHECK_EQUAL(original.kp_ir, decoded.kp_ir);
    CHECK_EQUAL(original.kd_ir, decoded.kd_ir);
}

TEST(MoveForwardTests, EncodeControlMaintainsFieldOrder)
{
    ControlConfig cfg;
    cfg.single_wall_target = 0;
    cfg.motor_speed = 1;
    cfg.kp_velocity = 2;
    cfg.kd_velocity = 3;
    cfg.kp_angle = 4;
    cfg.kd_angle = 5;
    cfg.pid_scale = 6;
    cfg.kp_ir = 7;
    cfg.kd_ir = 8;

    auto v{encode_control(cfg)};

    CHECK_EQUAL(0, v.at(0));
    CHECK_EQUAL(1, v.at(1));
    CHECK_EQUAL(2, v.at(2));
    CHECK_EQUAL(3, v.at(3));
    CHECK_EQUAL(4, v.at(4));
    CHECK_EQUAL(5, v.at(5));
    CHECK_EQUAL(6, v.at(6));
    CHECK_EQUAL(7, v.at(7));
    CHECK_EQUAL(8, v.at(8));
}

TEST(MoveForwardTests, GetControlBoundsHasCorrectSize)
{
    auto [low, high] = get_control_bounds();

    CHECK_EQUAL(9, low.size());
    CHECK_EQUAL(9, high.size());
}

TEST(MoveForwardTests, GetControlBoundsValuesAreCorrect)
{
    auto [low, high] = get_control_bounds();

    CHECK_EQUAL(0, low.at(0));
    CHECK_EQUAL(140, low.at(1));
    CHECK_EQUAL(0, low.at(2));
    CHECK_EQUAL(0, low.at(3));
    CHECK_EQUAL(0, low.at(4));
    CHECK_EQUAL(0, low.at(5));
    CHECK_EQUAL(16, low.at(6));
    CHECK_EQUAL(0, low.at(7));
    CHECK_EQUAL(0, low.at(8));

    CHECK_EQUAL(1024, high.at(0));
    CHECK_EQUAL(255, high.at(1));
    CHECK_EQUAL(2000, high.at(2));
    CHECK_EQUAL(2000, high.at(3));
    CHECK_EQUAL(2000, high.at(4));
    CHECK_EQUAL(2000, high.at(5));
    CHECK_EQUAL(512, high.at(6));
    CHECK_EQUAL(2000, high.at(7));
    CHECK_EQUAL(2000, high.at(8));
}

TEST(MoveForwardTests, GetControlBoundsAreDecodeSafe)
{
    auto [low, high] = get_control_bounds();

    auto low_cfg{decode_control(low)};
    auto high_cfg{decode_control(high)};

    CHECK_EQUAL(140, low_cfg.motor_speed);
    CHECK_EQUAL(255, high_cfg.motor_speed);
}

TEST(MoveForwardTests, RandomEnvironmentValuesWithinExpectedRanges)
{
    for (int i{0}; i < 100; i++) {
        auto e{generate_random_environment()};

        CHECK((e.dt >= 0.005) && (e.dt <= 0.01));
        CHECK((e.motor_speed_scale >= 0.9) && (e.motor_speed_scale <= 1.1));
        CHECK((e.motor1_variance >= -0.2) && (e.motor1_variance <= 0.2));
        CHECK((e.motor2_variance >= -0.2) && (e.motor2_variance <= 0.2));
        CHECK((e.slip_factor >= 0.9) && (e.slip_factor <= 1.1));
        CHECK((e.wheel_circumference_scale >= 0.9) && (e.wheel_circumference_scale <= 1.1));
        CHECK((e.wheel_base_scale >= 0.9) && (e.wheel_base_scale <= 1.1));
        CHECK((e.maze_size_scale >= 0.9) && (e.maze_size_scale <= 1.1));
        CHECK((e.ir_reading_scale >= 0.9) && (e.ir_reading_scale <= 1.1));
        CHECK((e.mouse_angle >= -(M_PI / 4)) && (e.mouse_angle <= (M_PI / 4)));
        CHECK((e.horizontal_position_variance >= -0.5) && (e.horizontal_position_variance <= 0.5));
        CHECK((e.vertical_position_variance >= -0.5) && (e.vertical_position_variance <= 0.5));
    }
}

TEST(MoveForwardTests, SimulationIsDeterministic)
{
    Config cfg{create_no_variance_config()};

    CHECK(are_results_equivalent_for_wall_mode(cfg, cfg, WallMode::NO_WALLS));
    CHECK(are_results_equivalent_for_wall_mode(cfg, cfg, WallMode::LEFT_WALL_ONLY));
    CHECK(are_results_equivalent_for_wall_mode(cfg, cfg, WallMode::BOTH_WALLS));
}

TEST(MoveForwardTests, NoVarianceProducesNearPerfectResults)
{
    Config cfg{create_no_variance_config()};

    for (WallMode m : WALL_MODES) {
        auto result{run_simulation(cfg, m)};

        CHECK(!was_there_collision_or_timeout(result));
        DOUBLES_EQUAL(0.0, result.total_angle_error, FLOAT_TOLERANCE);
        DOUBLES_EQUAL(0.0, result.total_horizontal_translation, FLOAT_TOLERANCE);

        constexpr double VERTICAL_TOLERANCE{5.0};
        DOUBLES_EQUAL(0.0, result.final_vertical_translation, VERTICAL_TOLERANCE);
    }
}

TEST(MoveForwardTests, DtAffectsResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2{create_no_variance_config()};
    cfg2.env_cfg.dt = 0.1;

    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::NO_WALLS));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::LEFT_WALL_ONLY));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::BOTH_WALLS));
}

TEST(MoveForwardTests, SpeedScaleAffectsResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2{create_no_variance_config()};
    cfg2.env_cfg.motor_speed_scale = 0.5;

    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::NO_WALLS));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::LEFT_WALL_ONLY));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::BOTH_WALLS));
}

TEST(MoveForwardTests, MotorVarianceAffectsResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2{create_no_variance_config()};
    cfg2.env_cfg.motor1_variance = 0.1;
    cfg2.env_cfg.motor2_variance = -0.1;

    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::NO_WALLS));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::LEFT_WALL_ONLY));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::BOTH_WALLS));
}

TEST(MoveForwardTests, SlipFactorAffectsResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2{create_no_variance_config()};
    cfg2.env_cfg.slip_factor = 0.5;

    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::NO_WALLS));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::LEFT_WALL_ONLY));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::BOTH_WALLS));
}

TEST(MoveForwardTests, WheelCircumferenceScaleAffectsResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2{create_no_variance_config()};
    cfg2.env_cfg.wheel_circumference_scale = 0.5;

    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::NO_WALLS));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::LEFT_WALL_ONLY));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::BOTH_WALLS));
}

TEST(MoveForwardTests, WheelBaseScaleAffectsResults)
{
    Config cfg1{create_no_variance_config()};
    cfg1.env_cfg.motor1_variance = 0.1;
    Config cfg2{cfg1};
    cfg2.env_cfg.wheel_base_scale = 0.5;

    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::NO_WALLS));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::LEFT_WALL_ONLY));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::BOTH_WALLS));
}

TEST(MoveForwardTests, MazeSizeScaleAffectsResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2{create_no_variance_config()};
    cfg2.env_cfg.maze_size_scale = 0.5;

    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::NO_WALLS));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::LEFT_WALL_ONLY));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::BOTH_WALLS));
}

TEST(MoveForwardTests, IRReadingScaleAffectsOneAndTwoWallResultsOnly)
{
    Config cfg1{create_no_variance_config()};
    cfg1.env_cfg.mouse_angle = M_PI / 32;
    cfg1.ctrl_cfg.kp_ir = 50;
    cfg1.env_cfg.ir_reading_scale = 0.5;
    Config cfg2{cfg1};
    cfg2.env_cfg.ir_reading_scale = 2.0;

    CHECK(are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::NO_WALLS));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::LEFT_WALL_ONLY));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::BOTH_WALLS));
}

TEST(MoveForwardTests, InitialAngleAffectsResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2{create_no_variance_config()};
    cfg2.env_cfg.mouse_angle = M_PI / 16;

    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::NO_WALLS));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::LEFT_WALL_ONLY));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::BOTH_WALLS));
}

TEST(MoveForwardTests, HorizontalOffsetAffectsOneAndTwoWallResultsOnly)
{
    Config cfg1{create_no_variance_config()};
    cfg1.ctrl_cfg.kp_ir = 50;
    Config cfg2{cfg1};
    cfg2.env_cfg.horizontal_position_variance = 0.5;

    CHECK(are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::NO_WALLS));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::LEFT_WALL_ONLY));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::BOTH_WALLS));
}

TEST(MoveForwardTests, SingleWallTargetAffectsOneWallResultsOnly)
{
    Config cfg1{create_no_variance_config()};
    cfg1.env_cfg.mouse_angle = M_PI / 32;
    cfg1.ctrl_cfg.kp_ir = 50;
    cfg1.ctrl_cfg.single_wall_target = 300;
    Config cfg2{cfg1};
    cfg2.ctrl_cfg.single_wall_target = 500;

    CHECK(are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::NO_WALLS));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::LEFT_WALL_ONLY));
    CHECK(are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::BOTH_WALLS));
}

TEST(MoveForwardTests, MotorSpeedAffectsResults)
{
    Config cfg1{create_no_variance_config()};
    cfg1.ctrl_cfg.motor_speed = 80;
    Config cfg2{create_no_variance_config()};
    cfg2.ctrl_cfg.motor_speed = 200;

    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::NO_WALLS));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::LEFT_WALL_ONLY));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::BOTH_WALLS));
}

TEST(MoveForwardTests, EncoderPDAffectsResults)
{
    Config cfg1{create_no_variance_config()};
    cfg1.env_cfg.motor1_variance = 0.1;
    Config cfg2{cfg1};
    cfg2.ctrl_cfg.kp_angle = 50;
    cfg2.ctrl_cfg.kd_angle = 10;

    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::NO_WALLS));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::LEFT_WALL_ONLY));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::BOTH_WALLS));
}

TEST(MoveForwardTests, IRControlAffectsOneAndTwoWallResultsOnly)
{
    Config cfg1{create_no_variance_config()};
    cfg1.env_cfg.mouse_angle = M_PI / 16;
    Config cfg2{cfg1};
    cfg2.ctrl_cfg.kp_ir = 100;
    cfg2.ctrl_cfg.kd_ir = 50;

    CHECK(are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::NO_WALLS));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::LEFT_WALL_ONLY));
    CHECK(!are_results_equivalent_for_wall_mode(cfg1, cfg2, WallMode::BOTH_WALLS));
}

TEST(MoveForwardTests, WallModesProduceDifferentResults)
{
    Config cfg{create_no_variance_config()};
    cfg.env_cfg.mouse_angle = M_PI / 32;
    cfg.ctrl_cfg.kp_ir = 50;

    auto no_wall{run_simulation(cfg, WallMode::NO_WALLS)};
    auto one_wall{run_simulation(cfg, WallMode::LEFT_WALL_ONLY)};
    auto two_wall{run_simulation(cfg, WallMode::BOTH_WALLS)};

    CHECK(!are_results_equivalent(no_wall, one_wall));
    CHECK(!are_results_equivalent(one_wall, two_wall));
}

IGNORE_TEST(MoveForwardTests, VisualizationDoesNotAffectResults)
{
    Config cfg{create_no_variance_config()};

    disable_visualization();
    auto no_wall_disabled{run_simulation(cfg, WallMode::NO_WALLS)};
    auto one_wall_disabled{run_simulation(cfg, WallMode::LEFT_WALL_ONLY)};
    auto two_wall_disabled{run_simulation(cfg, WallMode::BOTH_WALLS)};

    enable_visualization("visualization-does-not-affect-results");
    auto no_wall_enabled{run_simulation(cfg, WallMode::NO_WALLS)};
    auto one_wall_enabled{run_simulation(cfg, WallMode::LEFT_WALL_ONLY)};
    auto two_wall_enabled{run_simulation(cfg, WallMode::BOTH_WALLS)};

    CHECK(are_results_equivalent(no_wall_disabled, no_wall_enabled));
    CHECK(are_results_equivalent(one_wall_disabled, one_wall_enabled));
    CHECK(are_results_equivalent(one_wall_disabled, two_wall_enabled));
}
