/*================================ FILE INFO =================================*/
/* Filename           : test_side_wall_detection.cpp                          */
/*                                                                            */
/* Test implementation for side_wall_detection.cpp                            */
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
#include "simulation_common.hpp"
#include "side_wall_detection.hpp"

using namespace side_wall_detection;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
constexpr double FLOAT_TOLERANCE{1e-6};

ControlConfig ctr_lower{};
ControlConfig ctr_upper{};
EnvironmentConfig env_lower{};
EnvironmentConfig env_upper{};

void set_local_ctr_bound_variables(void)
{
    ctr_lower.reading_threshold = 0;

    ctr_upper.reading_threshold = 1024;
}

void set_local_env_bound_variables(void)
{
    env_lower.maze_size_scale = 0.9;
    env_lower.ir_reading_scale = 0.9;
    env_lower.mouse_angle = -(M_PI / 4);
    env_lower.horizontal_position_variance = -0.9;
    env_lower.vertical_position_variance = -0.9;
    env_lower.total_steps = 100;

    env_upper.maze_size_scale = 1.1;
    env_upper.ir_reading_scale = 1.1;
    env_upper.mouse_angle = M_PI / 4;
    env_upper.horizontal_position_variance = 0.9;
    env_upper.vertical_position_variance = 0.9;
    env_upper.total_steps = 100;
}

void set_config_bounds(void)
{
    set_ctr_config_bounds(ctr_lower, ctr_upper);
    set_env_config_bounds(env_lower, env_upper);
}

void reset_local_and_assigned_config_bounds(void)
{
    ctr_lower = {};
    ctr_upper = {};
    env_lower = {};
    env_upper = {};
    reset_all_config_bounds();
}

Config create_no_variance_config(void)
{
    Config cfg{};
    cfg.env_cfg.maze_size_scale = 1.0;
    cfg.env_cfg.ir_reading_scale = 1.0;
    cfg.env_cfg.mouse_angle = 0.0;
    cfg.env_cfg.horizontal_position_variance = 0.0;
    cfg.env_cfg.vertical_position_variance = 0.0;
    cfg.env_cfg.total_steps = 100;
    cfg.ctrl_cfg.reading_threshold = 80u; /* arbitrary threshold */

    return cfg;
}

bool are_results_equivalent(const Result& r1, const Result& r2)
{
    if (r1.wall_absent_at_step.size() != r2.wall_absent_at_step.size()) {
        return false;
    }
    if (r1.wall_present_at_step.size() != r2.wall_present_at_step.size()) {
        return false;
    }

    for (size_t i{0}; i < r1.wall_absent_at_step.size(); ++i) {
        if (r1.wall_absent_at_step.at(i) != r2.wall_absent_at_step.at(i)) {
            return false;
        }
        if (r1.wall_present_at_step.at(i) != r2.wall_present_at_step.at(i)) {
            return false;
        }
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
TEST_GROUP(SideWallDetectionTests)
{
    void setup() override
    {
        reset_local_and_assigned_config_bounds();
        disable_visualization();
    }

    void teardown() override
    {
        disable_visualization();
        reset_local_and_assigned_config_bounds();
    }
};

/*============================================================================*/
/*                                    Tests                                   */
/*============================================================================*/
TEST(SideWallDetectionTests, ResetAllConfigBoundsClearsBounds)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    reset_all_config_bounds();

    auto [low, high] = get_control_bounds();

    for (size_t i{0}; i < low.size(); ++i) {
        CHECK_EQUAL(0.0, low.at(i));
        CHECK_EQUAL(0.0, high.at(i));
    }
}

TEST(SideWallDetectionTests, EncodeDecodeControlRoundTrip)
{
    ControlConfig original;
    original.reading_threshold = 123u;

    auto encoded{encode_control(original)};
    auto decoded{decode_control(encoded)};

    CHECK_EQUAL(original.reading_threshold, decoded.reading_threshold);
}

TEST(SideWallDetectionTests, EncodeControlMaintainsFieldOrder)
{
    ControlConfig cfg;
    cfg.reading_threshold = 0;

    auto v{encode_control(cfg)};

    CHECK_EQUAL(0, v.at(0));
}

TEST(SideWallDetectionTests, GetControlBoundsHasCorrectSize)
{
    auto [low, high] = get_control_bounds();

    CHECK_EQUAL(1, low.size());
    CHECK_EQUAL(1, high.size());
}

TEST(SideWallDetectionTests, GetControlBoundsValuesAreCorrect)
{
    set_local_ctr_bound_variables();
    set_config_bounds();

    auto [low, high] = get_control_bounds();

    CHECK_EQUAL(0, low.at(0));

    CHECK_EQUAL(1024, high.at(0));
}

TEST(SideWallDetectionTests, GetControlBoundsAreDecodeSafe)
{
    set_local_ctr_bound_variables();
    set_config_bounds();

    auto [low, high] = get_control_bounds();

    auto low_cfg{decode_control(low)};
    auto high_cfg{decode_control(high)};

    CHECK_EQUAL(0, low_cfg.reading_threshold);
    CHECK_EQUAL(1024, high_cfg.reading_threshold);
}

TEST(SideWallDetectionTests, RandomEnvironmentValuesWithinExpectedRanges)
{
    set_local_env_bound_variables();
    set_config_bounds();

    for (int i{0}; i < 100; i++) {
        auto e{generate_random_environment()};

        CHECK((e.maze_size_scale >= 0.9) && (e.maze_size_scale <= 1.1));
        CHECK((e.ir_reading_scale >= 0.9) && (e.ir_reading_scale <= 1.1));
        CHECK((e.mouse_angle >= -(M_PI / 4)) && (e.mouse_angle <= M_PI / 4));
        CHECK((e.horizontal_position_variance >= -0.9) && (e.horizontal_position_variance <= 0.9));
        CHECK((e.vertical_position_variance >= -0.9) && (e.vertical_position_variance <= 0.9));
        CHECK(e.total_steps == 100);
    }
}

TEST(SideWallDetectionTests, SimulationHandlesZeroSteps)
{
    Config cfg{create_no_variance_config()};
    cfg.env_cfg.total_steps = 0;

    auto result{run_simulation(cfg)};

    CHECK_EQUAL(0, result.wall_absent_at_step.size());
    CHECK_EQUAL(0, result.wall_present_at_step.size());
}

TEST(SideWallDetectionTests, WallAbsentAllFalseWhenThresholdIsZero)
{
    Config cfg{create_no_variance_config()};
    cfg.ctrl_cfg.reading_threshold = 0u; /* nothing should pass */

    auto result{run_simulation(cfg)};

    for (bool v : result.wall_absent_at_step) {
        CHECK_FALSE(v);
    }
}

TEST(SideWallDetectionTests, WallPresentAllTrueWhenThresholdIsZero)
{
    Config cfg{create_no_variance_config()};
    cfg.ctrl_cfg.reading_threshold = 0u; /* everything should pass */

    auto result{run_simulation(cfg)};

    for (bool v : result.wall_present_at_step) {
        CHECK(v);
    }
}

TEST(SideWallDetectionTests, IdenticalConfigsProduceIdenticalResults)
{
    Config cfg{create_no_variance_config()};

    auto r1{run_simulation(cfg)};
    auto r2{run_simulation(cfg)};

    CHECK(are_results_equivalent(r1, r2));
}

TEST(SideWallDetectionTests, MazeSizeScaleChangesResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2{cfg1};
    cfg2.env_cfg.maze_size_scale = 10.0;

    auto r1{run_simulation(cfg1)};
    auto r2{run_simulation(cfg2)};

    CHECK(!are_results_equivalent(r1, r2));
}

TEST(SideWallDetectionTests, IrReadingScaleChangesResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2{cfg1};
    cfg2.env_cfg.ir_reading_scale = 0.5;

    auto r1{run_simulation(cfg1)};
    auto r2{run_simulation(cfg2)};

    CHECK(!are_results_equivalent(r1, r2));
}

TEST(SideWallDetectionTests, ZeroIrReadingScaleCollapsesToAllTrueWhenThresholdIsMax)
{
    Config cfg{create_no_variance_config()};
    cfg.ctrl_cfg.reading_threshold = 1024u;

    auto result{run_simulation(cfg)};

    for (bool v : result.wall_absent_at_step) {
        CHECK(v);
    }
}

TEST(SideWallDetectionTests, MouseAngleChangesResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2{cfg1};
    cfg2.env_cfg.mouse_angle = M_PI / 2;

    auto r1{run_simulation(cfg1)};
    auto r2{run_simulation(cfg2)};

    CHECK(!are_results_equivalent(r1, r2));
}

TEST(SideWallDetectionTests, HorizontalVarianceChangesResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2{cfg1};
    cfg2.env_cfg.horizontal_position_variance = -0.9;

    auto r1{run_simulation(cfg1)};
    auto r2{run_simulation(cfg2)};

    CHECK(!are_results_equivalent(r1, r2));
}

TEST(SideWallDetectionTests, VerticalVarianceChangesResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2{cfg1};
    cfg2.env_cfg.vertical_position_variance = -0.9;

    auto r1{run_simulation(cfg1)};
    auto r2{run_simulation(cfg2)};

    CHECK(!are_results_equivalent(r1, r2));
}

IGNORE_TEST(SideWallDetectionTests, VisualizationDoesNotAffectResults)
{
    Config cfg{create_no_variance_config()};

    disable_visualization();
    auto r1{run_simulation(cfg)};

    enable_visualization();
    auto r2{run_simulation(cfg)};

    CHECK(are_results_equivalent(r1, r2));
}
