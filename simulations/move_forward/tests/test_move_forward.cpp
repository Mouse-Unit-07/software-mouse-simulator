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

    cfg.env_cfg.dt = {0.001};
    cfg.env_cfg.motor_speed_scale = {1.0};
    cfg.env_cfg.motor1_variance = {0.0};
    cfg.env_cfg.motor2_variance = {0.0};
    cfg.env_cfg.slip_factor = {1.0};
    cfg.env_cfg.wheel_circumference_scale = {1.0};
    cfg.env_cfg.wheel_base_scale = {1.0};
    cfg.env_cfg.maze_size_scale = {1.0};
    cfg.env_cfg.ir_reading_scale = {1.0};
    cfg.env_cfg.mouse_angle = {0.0};
    cfg.env_cfg.horizontal_position_variance = {0.0};
    cfg.env_cfg.vertical_position_variance = {0.0};

    cfg.ctrl_cfg.single_wall_target = {407u};
    cfg.ctrl_cfg.motor_speed = {120u};
    cfg.ctrl_cfg.kp = {0};
    cfg.ctrl_cfg.kd = {0};
    cfg.ctrl_cfg.pid_shift = {0};
    cfg.ctrl_cfg.kp_ir = {0};
    cfg.ctrl_cfg.kd_ir = {0};

    return cfg;
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

SingleCaseResult create_single_case_result(double time, double angle, double horizontal_translation,
                                           double vertical_translation, bool collision = false,
                                           bool timeout = false)
{
    SingleCaseResult out;
    out.total_time = time;
    out.total_angle_error = angle;
    out.total_horizontal_translation = horizontal_translation;
    out.final_vertical_translation = vertical_translation;
    out.collision = collision;
    out.timeout = timeout;

    return out;
}

Result create_result(const SingleCaseResult& no, const SingleCaseResult& one,
                     const SingleCaseResult& two)
{
    Result out;
    out.no_wall = no;
    out.one_wall = one;
    out.two_wall = two;

    return out;
}

Config create_custom_config(uint32_t target = 100, uint8_t speed = 100, int kp = 1, int kd = 1,
                            int shift = 0, int kp_ir = 0, int kd_ir = 0)
{
    Config c{};
    c.ctrl_cfg.single_wall_target = target;
    c.ctrl_cfg.motor_speed = speed;
    c.ctrl_cfg.kp = kp;
    c.ctrl_cfg.kd = kd;
    c.ctrl_cfg.pid_shift = shift;
    c.ctrl_cfg.kp_ir = kp_ir;
    c.ctrl_cfg.kd_ir = kd_ir;
    return c;
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
    original.kp = 1000;
    original.kd = -250;
    original.pid_shift = 6;
    original.kp_ir = 100;
    original.kd_ir = 200;

    auto encoded{encode_control(original)};
    auto decoded{decode_control(encoded)};

    CHECK_EQUAL(original.single_wall_target, decoded.single_wall_target);
    CHECK_EQUAL(original.motor_speed, decoded.motor_speed);
    CHECK_EQUAL(original.kp, decoded.kp);
    CHECK_EQUAL(original.kd, decoded.kd);
    CHECK_EQUAL(original.pid_shift, decoded.pid_shift);
    CHECK_EQUAL(original.kp_ir, decoded.kp_ir);
    CHECK_EQUAL(original.kd_ir, decoded.kd_ir);
}

TEST(MoveForwardTests, EncodeControlMaintainsFieldOrder)
{
    ControlConfig cfg;
    cfg.single_wall_target = 0;
    cfg.motor_speed = 1;
    cfg.kp = 2;
    cfg.kd = 3;
    cfg.pid_shift = 4;
    cfg.kp_ir = 5;
    cfg.kd_ir = 6;

    auto v{encode_control(cfg)};

    CHECK_EQUAL(0, v.at(0));
    CHECK_EQUAL(1, v.at(1));
    CHECK_EQUAL(2, v.at(2));
    CHECK_EQUAL(3, v.at(3));
    CHECK_EQUAL(4, v.at(4));
    CHECK_EQUAL(5, v.at(5));
    CHECK_EQUAL(6, v.at(6));
}

TEST(MoveForwardTests, GetControlBoundsHasCorrectSize)
{
    auto [low, high] = get_control_bounds();

    CHECK_EQUAL(7, low.size());
    CHECK_EQUAL(7, high.size());
}

TEST(MoveForwardTests, GetControlBoundsValuesAreCorrect)
{
    auto [low, high] = get_control_bounds();

    CHECK_EQUAL(0, low.at(0));
    CHECK_EQUAL(100, low.at(1));
    CHECK_EQUAL(0, low.at(2));
    CHECK_EQUAL(0, low.at(3));

    CHECK_EQUAL(1024, high.at(0));
    CHECK_EQUAL(255, high.at(1));
    CHECK_EQUAL(2000, high.at(2));
    CHECK_EQUAL(2000, high.at(3));
}

TEST(MoveForwardTests, GetControlBoundsAreDecodeSafe)
{
    auto [low, high] = get_control_bounds();

    auto low_cfg{decode_control(low)};
    auto high_cfg{decode_control(high)};

    CHECK_EQUAL(100, low_cfg.motor_speed);
    CHECK_EQUAL(255, high_cfg.motor_speed);
}

TEST(MoveForwardTests, RandomEnvironmentValuesWithinExpectedRanges)
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

    cfg.env_cfg.dt = 0.01;
    auto r1{run_simulation(cfg)};

    cfg.env_cfg.dt = 0.1;
    auto r2{run_simulation(cfg)};

    CHECK(!are_results_equivalent(r1, r2));
}

TEST(MoveForwardTests, SpeedScaleAffectsResults)
{
    Config cfg{create_no_variance_config()};

    auto no_speed_scale{run_simulation(cfg)};

    cfg.env_cfg.motor_speed_scale = 0.5;
    auto with_speed_scale{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_speed_scale, with_speed_scale));
}

TEST(MoveForwardTests, MotorVarianceAffectsResults)
{
    Config cfg{create_no_variance_config()};

    auto no_variance{run_simulation(cfg)};

    cfg.env_cfg.motor1_variance = 0.1;
    cfg.env_cfg.motor2_variance = -0.1;
    auto with_variance{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_variance, with_variance));
}

TEST(MoveForwardTests, SlipFactorAffectsResults)
{
    Config cfg{create_no_variance_config()};

    auto no_slip_factor{run_simulation(cfg)};

    cfg.env_cfg.slip_factor = 0.5;
    auto with_slip_factor{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_slip_factor, with_slip_factor));
}

TEST(MoveForwardTests, WheelCircumferenceScaleAffectsResults)
{
    Config cfg{create_no_variance_config()};

    auto no_circumference_scale{run_simulation(cfg)};

    cfg.env_cfg.wheel_circumference_scale = 0.5;
    auto with_circumference_scale{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_circumference_scale, with_circumference_scale));
}

TEST(MoveForwardTests, WheelBaseScaleAffectsResults)
{
    Config cfg{create_no_variance_config()};
    cfg.env_cfg.motor1_variance = 0.1;

    auto no_base_scale{run_simulation(cfg)};

    cfg.env_cfg.wheel_base_scale = 0.5;
    auto with_base_scale{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_base_scale, with_base_scale));
}

TEST(MoveForwardTests, MazeSizeScaleAffectsResults)
{
    Config cfg{create_no_variance_config()};

    auto no_size_scale{run_simulation(cfg)};

    cfg.env_cfg.maze_size_scale = 0.5;
    auto with_size_scale{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_size_scale, with_size_scale));
}

TEST(MoveForwardTests, IRReadingScaleAffectsResults)
{
    Config cfg{create_no_variance_config()};
    cfg.env_cfg.mouse_angle = M_PI / 32;
    cfg.ctrl_cfg.kp_ir = 50;

    cfg.env_cfg.ir_reading_scale = 0.5;
    auto r1{run_simulation(cfg)};

    cfg.env_cfg.ir_reading_scale = 2.0;
    auto r2{run_simulation(cfg)};

    CHECK(!are_results_equivalent(r1, r2));
}

TEST(MoveForwardTests, InitialAngleAffectsResults)
{
    Config cfg{create_no_variance_config()};

    auto straight{run_simulation(cfg)};

    cfg.env_cfg.mouse_angle = M_PI / 16;
    auto angled{run_simulation(cfg)};

    CHECK(!are_results_equivalent(straight, angled));
}

TEST(MoveForwardTests, HorizontalOffsetAffectsResults)
{
    Config cfg{create_no_variance_config()};
    cfg.ctrl_cfg.kp_ir = 50;

    auto no_offset{run_simulation(cfg)};

    cfg.env_cfg.horizontal_position_variance = 0.5;
    auto with_offset{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_offset, with_offset));
}

TEST(MoveForwardTests, SingleWallTargetAffectsResults)
{
    Config cfg{create_no_variance_config()};
    cfg.env_cfg.mouse_angle = M_PI / 32;
    cfg.ctrl_cfg.kp_ir = 50;

    cfg.ctrl_cfg.single_wall_target = 300;
    auto r1{run_simulation(cfg)};

    cfg.ctrl_cfg.single_wall_target = 500;
    auto r2{run_simulation(cfg)};

    CHECK(!are_single_case_results_equivalent(r1.one_wall, r2.one_wall));
}

TEST(MoveForwardTests, MotorSpeedAffectsTotalTime)
{
    Config cfg{create_no_variance_config()};

    cfg.ctrl_cfg.motor_speed = 80;
    auto slow{run_simulation(cfg)};

    cfg.ctrl_cfg.motor_speed = 200;
    auto fast{run_simulation(cfg)};

    CHECK(fast.no_wall.total_time < slow.no_wall.total_time);
}

TEST(MoveForwardTests, EncoderPDAffectsResults)
{
    Config cfg{create_no_variance_config()};
    cfg.env_cfg.motor1_variance = 0.1;

    auto no_pd{run_simulation(cfg)};

    cfg.ctrl_cfg.kp = 50;
    cfg.ctrl_cfg.kd = 10;
    auto with_pd{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_pd, with_pd));
}

TEST(MoveForwardTests, IRControlAffectsJustOneAndTwoWallResults)
{
    Config cfg{create_no_variance_config()};
    cfg.env_cfg.mouse_angle = M_PI / 16;

    auto no_ir_control{run_simulation(cfg)};

    cfg.ctrl_cfg.kp_ir = 100;
    cfg.ctrl_cfg.kd_ir = 50;
    auto with_ir_control{run_simulation(cfg)};

    CHECK(are_single_case_results_equivalent(no_ir_control.no_wall, with_ir_control.no_wall));
    CHECK(!are_single_case_results_equivalent(no_ir_control.one_wall, with_ir_control.one_wall));
    CHECK(!are_single_case_results_equivalent(no_ir_control.two_wall, with_ir_control.two_wall));
}

TEST(MoveForwardTests, WallModesProduceDifferentResults)
{
    Config cfg{create_no_variance_config()};
    cfg.env_cfg.mouse_angle = M_PI / 32;
    cfg.ctrl_cfg.kp_ir = 50;

    auto result{run_simulation(cfg)};

    CHECK(!are_single_case_results_equivalent(result.no_wall, result.one_wall));
    CHECK(!are_single_case_results_equivalent(result.one_wall, result.two_wall));
}

TEST(MoveForwardTests, VisualizationDoesNotAffectResults)
{
    Config cfg{create_no_variance_config()};

    disable_visualization();
    auto r1{run_simulation(cfg)};

    enable_visualization("visualization-does-not-affect-results");
    auto r2{run_simulation(cfg)};

    CHECK(are_results_equivalent(r1, r2));
}
