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

    cfg.dt = {0.001};
    cfg.motor_speed_scale = {1.0};
    cfg.motor1_variance = {0.0};
    cfg.motor2_variance = {0.0};
    cfg.slip_factor = {1.0};
    cfg.wheel_circumference_scale = {1.0};
    cfg.wheel_base_scale = {1.0};
    cfg.maze_size_scale = {1.0};
    cfg.ir_reading_scale = {1.0};
    cfg.mouse_angle = {0.0};
    cfg.horizontal_position_variance = {0.0};
    cfg.vertical_position_variance = {0.0};

    cfg.single_wall_target = {407u};
    cfg.motor_speed = {120u};
    cfg.kp = {0};
    cfg.kd = {0};
    cfg.pid_shift = {0};
    cfg.kp_ir = {0};
    cfg.kd_ir = {0};

    return cfg;
}

ConfigSweeper create_no_variance_sweeper(void)
{
    ConfigSweeper sweeper;

    sweeper.dt = {0.001};
    sweeper.motor_speed_scale = {1.0};
    sweeper.motor1_variance = {0.0};
    sweeper.motor2_variance = {0.0};
    sweeper.slip_factor = {1.0};
    sweeper.wheel_circumference_scale = {1.0};
    sweeper.wheel_base_scale = {1.0};
    sweeper.maze_size_scale = {1.0};
    sweeper.ir_reading_scale = {1.0};
    sweeper.mouse_angle = {0.0};
    sweeper.horizontal_position_variance = {0.0};
    sweeper.vertical_position_variance = {0.0};

    sweeper.single_wall_target = {407u};
    sweeper.motor_speed = {120u};
    sweeper.kp = {0};
    sweeper.kd = {0};
    sweeper.pid_shift = {0};
    sweeper.kp_ir = {0};
    sweeper.kd_ir = {0};

    return sweeper;
}

bool was_there_collision_or_timeout(const Result& result)
{
    if (result.no_wall.collision || result.one_wall.collision || result.two_wall.collision
        || result.no_wall.timeout || result.one_wall.timeout || result.two_wall.timeout) {
        return true;
    }

    return false;
}

bool are_single_case_results_equivalent(const SingleCaseResult& r1, const SingleCaseResult& r2)
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

bool are_results_equivalent(const Result& r1, const Result& r2)
{
    if (!are_single_case_results_equivalent(r1.no_wall, r2.no_wall)) {
        return false;
    }
    if (!are_single_case_results_equivalent(r1.one_wall, r2.one_wall)) {
        return false;
    }
    if (!are_single_case_results_equivalent(r1.two_wall, r2.two_wall)) {
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

    }

    void teardown() override
    {

    }
};

/*============================================================================*/
/*                                    Tests                                   */
/*============================================================================*/
TEST(MoveForwardTests, ConfigSweeperProducesFirstValue)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};

    CHECK(sweeper.next());

    auto cfg{sweeper.value()};

    CHECK_EQUAL(120, cfg.motor_speed);
    CHECK_EQUAL(407u, cfg.single_wall_target);
    DOUBLES_EQUAL(0.0, cfg.motor1_variance, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(1.0, cfg.ir_reading_scale, FLOAT_TOLERANCE);
}

TEST(MoveForwardTests, ConfigSweeperIteratesAllCombinations)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};
    sweeper.motor_speed = {100, 200};
    sweeper.kp = {1, 2};

    int count{0};

    while (sweeper.next()) {
        sweeper.value();
        count++;
    }

    CHECK_EQUAL(4, count); /* 2 motor_speed * 2 kp */
}

TEST(MoveForwardTests, ConfigSweeperStopsAtEnd)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};

    CHECK(sweeper.next());
    CHECK_FALSE(sweeper.next());
}

TEST(MoveForwardTests, ConfigSweeperOrderIsStable)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};
    sweeper.kp = {1, 2};
    sweeper.kd = {10, 20};

    std::vector<std::pair<int, int>> seen;

    while (sweeper.next()) {
        auto cfg{sweeper.value()};
        seen.emplace_back(cfg.kp, cfg.kd);
    }

    CHECK_EQUAL(4, seen.size());

    /* Expected order: kp outer, kd inner */
    CHECK(seen.at(0) == std::make_pair(1, 10));
    CHECK(seen.at(1) == std::make_pair(1, 20));
    CHECK(seen.at(2) == std::make_pair(2, 10));
    CHECK(seen.at(3) == std::make_pair(2, 20));
}

TEST(MoveForwardTests, SimulationIsDeterministic)
{
    Config cfg{create_no_variance_config()};

    auto r1{run_simulation(cfg)};
    auto r2{run_simulation(cfg)};

    CHECK(are_results_equivalent(r1, r2));
}

TEST(MoveForwardTests, NoVarianceProducesNearPerfectResults)
{
    Config cfg{create_no_variance_config()};

    auto result{run_simulation(cfg)};

    CHECK(!was_there_collision_or_timeout(result));
    DOUBLES_EQUAL(0.0, result.no_wall.total_angle_error, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0, result.one_wall.total_angle_error, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0, result.two_wall.total_angle_error, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0, result.no_wall.total_horizontal_translation, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0, result.one_wall.total_horizontal_translation, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0, result.two_wall.total_horizontal_translation, FLOAT_TOLERANCE);

    constexpr double VERTICAL_TOLERANCE{5.0};
    DOUBLES_EQUAL(0.0, result.no_wall.final_vertical_translation, VERTICAL_TOLERANCE);
    DOUBLES_EQUAL(0.0, result.one_wall.final_vertical_translation, VERTICAL_TOLERANCE);
    DOUBLES_EQUAL(0.0, result.two_wall.final_vertical_translation, VERTICAL_TOLERANCE);
}

TEST(MoveForwardTests, DtAffectsResults)
{
    Config cfg{create_no_variance_config()};

    cfg.dt = 0.01;
    auto r1{run_simulation(cfg)};

    cfg.dt = 0.1;
    auto r2{run_simulation(cfg)};

    CHECK(!are_results_equivalent(r1, r2));
}

TEST(MoveForwardTests, SpeedScaleAffectsResults)
{
    Config cfg{create_no_variance_config()};

    auto no_speed_scale{run_simulation(cfg)};

    cfg.motor_speed_scale = 0.5;
    auto with_speed_scale{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_speed_scale, with_speed_scale));
}

TEST(MoveForwardTests, MotorVarianceAffectsResults)
{
    Config cfg{create_no_variance_config()};

    auto no_variance{run_simulation(cfg)};

    cfg.motor1_variance = 0.1;
    cfg.motor2_variance = -0.1;
    auto with_variance{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_variance, with_variance));
}

TEST(MoveForwardTests, SlipFactorAffectsResults)
{
    Config cfg{create_no_variance_config()};

    auto no_slip_factor{run_simulation(cfg)};

    cfg.slip_factor = 0.5;
    auto with_slip_factor{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_slip_factor, with_slip_factor));
}

TEST(MoveForwardTests, WheelCircumferenceScaleAffectsResults)
{
    Config cfg{create_no_variance_config()};

    auto no_circumference_scale{run_simulation(cfg)};

    cfg.wheel_circumference_scale = 0.5;
    auto with_circumference_scale{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_circumference_scale, with_circumference_scale));
}

TEST(MoveForwardTests, WheelBaseScaleAffectsResults)
{
    Config cfg{create_no_variance_config()};
    cfg.motor1_variance = 0.1;

    auto no_base_scale{run_simulation(cfg)};

    cfg.wheel_base_scale = 0.5;
    auto with_base_scale{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_base_scale, with_base_scale));
}

TEST(MoveForwardTests, MazeSizeScaleAffectsResults)
{
    Config cfg{create_no_variance_config()};

    auto no_size_scale{run_simulation(cfg)};

    cfg.maze_size_scale = 0.5;
    auto with_size_scale{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_size_scale, with_size_scale));
}

TEST(MoveForwardTests, IRReadingScaleAffectsResults)
{
    Config cfg{create_no_variance_config()};
    cfg.mouse_angle = M_PI / 32;
    cfg.kp_ir = 50;

    cfg.ir_reading_scale = 0.5;
    auto r1{run_simulation(cfg)};

    cfg.ir_reading_scale = 2.0;
    auto r2{run_simulation(cfg)};

    CHECK(!are_results_equivalent(r1, r2));
}

TEST(MoveForwardTests, InitialAngleAffectsResults)
{
    Config cfg{create_no_variance_config()};

    auto straight{run_simulation(cfg)};

    cfg.mouse_angle = M_PI / 16;
    auto angled{run_simulation(cfg)};

    CHECK(!are_results_equivalent(straight, angled));
}

TEST(MoveForwardTests, HorizontalOffsetAffectsResults)
{
    Config cfg{create_no_variance_config()};
    cfg.kp_ir = 50;

    auto no_offset{run_simulation(cfg)};

    cfg.horizontal_position_variance = 0.5;
    auto with_offset{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_offset, with_offset));
}

TEST(MoveForwardTests, SingleWallTargetAffectsResults)
{
    Config cfg{create_no_variance_config()};
    cfg.mouse_angle = M_PI / 32;
    cfg.kp_ir = 50;

    cfg.single_wall_target = 300;
    auto r1{run_simulation(cfg)};

    cfg.single_wall_target = 500;
    auto r2{run_simulation(cfg)};

    CHECK(!are_single_case_results_equivalent(r1.one_wall, r2.one_wall));
}

TEST(MoveForwardTests, MotorSpeedAffectsTotalTime)
{
    Config cfg{create_no_variance_config()};

    cfg.motor_speed = 80;
    auto slow{run_simulation(cfg)};

    cfg.motor_speed = 200;
    auto fast{run_simulation(cfg)};

    CHECK(fast.no_wall.total_time < slow.no_wall.total_time);
}

TEST(MoveForwardTests, EncoderPDAffectsResults)
{
    Config cfg{create_no_variance_config()};
    cfg.motor1_variance = 0.1;

    auto no_pd{run_simulation(cfg)};

    cfg.kp = 50;
    cfg.kd = 10;
    auto with_pd{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_pd, with_pd));
}

TEST(MoveForwardTests, IRControlAffectsJustOneAndTwoWallResults)
{
    Config cfg{create_no_variance_config()};
    cfg.mouse_angle = M_PI / 16;

    auto no_ir_control{run_simulation(cfg)};

    cfg.kp_ir = 100;
    cfg.kd_ir = 50;
    auto with_ir_control{run_simulation(cfg)};

    CHECK(are_single_case_results_equivalent(no_ir_control.no_wall, with_ir_control.no_wall));
    CHECK(!are_single_case_results_equivalent(no_ir_control.one_wall, with_ir_control.one_wall));
    CHECK(!are_single_case_results_equivalent(no_ir_control.two_wall, with_ir_control.two_wall));
}

TEST(MoveForwardTests, WallModesProduceDifferentResults)
{
    Config cfg{create_no_variance_config()};
    cfg.mouse_angle = M_PI / 32;
    cfg.kp_ir = 50;

    auto result{run_simulation(cfg)};

    CHECK(!are_single_case_results_equivalent(result.no_wall, result.one_wall));
    CHECK(!are_single_case_results_equivalent(result.one_wall, result.two_wall));
}
