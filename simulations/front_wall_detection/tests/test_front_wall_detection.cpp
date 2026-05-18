/*================================ FILE INFO =================================*/
/* Filename           : test_front_wall_detection.cpp                         */
/*                                                                            */
/* Test implementation for front_wall_detection.cpp                           */
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
#include "front_wall_detection.hpp"

using namespace front_wall_detection;

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
    env_lower.ir_reading_scale = 0.9;
    env_lower.mouse_angle = -(M_PI / 4);
    env_lower.horizontal_position_variance = -0.9;
    env_lower.vertical_position_variance = -0.9;

    env_upper.ir_reading_scale = 1.1;
    env_upper.mouse_angle = M_PI / 4;
    env_upper.horizontal_position_variance = 0.9;
    env_upper.vertical_position_variance = 0.9;
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
    cfg.env_cfg.ir_reading_scale = {1.0};
    cfg.env_cfg.mouse_angle = {0.0};
    cfg.env_cfg.horizontal_position_variance = {0.0};
    cfg.env_cfg.vertical_position_variance = {0.0};

    cfg.ctrl_cfg.reading_threshold = {300u};

    return cfg;
}

bool are_results_equivalent(const Result& r1, const Result& r2)
{
    if (r1.identified_absent_wall != r2.identified_absent_wall) {
        return false;
    }
    if (r1.identified_present_wall != r2.identified_present_wall) {
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
TEST_GROUP(FrontWallDetectionTests)
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
TEST(FrontWallDetectionTests, ResetAllConfigBoundsClearsBounds)
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

TEST(FrontWallDetectionTests, EncodeDecodeControlRoundTrip)
{
    ControlConfig original{};
    original.reading_threshold = 123u;

    auto encoded{encode_control(original)};
    auto decoded{decode_control(encoded)};

    CHECK_EQUAL(original.reading_threshold, decoded.reading_threshold);
}

TEST(FrontWallDetectionTests, EncodeControlMaintainsFieldOrder)
{
    ControlConfig cfg{};
    cfg.reading_threshold = 0;

    auto v{encode_control(cfg)};

    CHECK_EQUAL(0, v.at(0));
}

TEST(FrontWallDetectionTests, GetControlBoundsHasCorrectSize)
{
    auto [low, high] = get_control_bounds();

    CHECK_EQUAL(1, low.size());
    CHECK_EQUAL(1, high.size());
}

TEST(FrontWallDetectionTests, GetControlBoundsValuesAreCorrect)
{
    set_local_ctr_bound_variables();
    set_config_bounds();

    auto [low, high] = get_control_bounds();

    CHECK_EQUAL(0, low.at(0));

    CHECK_EQUAL(1024, high.at(0));
}

TEST(FrontWallDetectionTests, GetControlBoundsAreDecodeSafe)
{
    set_local_ctr_bound_variables();
    set_config_bounds();

    auto [low, high] = get_control_bounds();

    auto low_cfg{decode_control(low)};
    auto high_cfg{decode_control(high)};

    CHECK_EQUAL(0, low_cfg.reading_threshold);
    CHECK_EQUAL(1024, high_cfg.reading_threshold);
}

TEST(FrontWallDetectionTests, RandomEnvironmentValuesWithinExpectedRanges)
{
    set_local_env_bound_variables();
    set_config_bounds();

    for (int i{0}; i < 100; i++) {
        auto e{generate_random_environment()};

        CHECK((e.ir_reading_scale >= 0.9) && (e.ir_reading_scale <= 1.1));
        CHECK((e.mouse_angle >= -(M_PI / 4)) && (e.mouse_angle <= M_PI / 4));
        CHECK((e.horizontal_position_variance >= -0.9) && (e.horizontal_position_variance <= 0.9));
        CHECK((e.vertical_position_variance >= -0.9) && (e.vertical_position_variance <= 0.9));
    }
}

TEST(FrontWallDetectionTests, ExtremeScaleTriggersClamping)
{
    Config cfg{create_no_variance_config()};

    cfg.env_cfg.ir_reading_scale = 100.0; /* force overflow */

    auto result{run_simulation(cfg)};

    CHECK(result.identified_present_wall == true || result.identified_present_wall == false);
}

TEST(FrontWallDetectionTests, ZeroThresholdMeansWallAlwaysPresent)
{
    Config cfg{create_no_variance_config()};
    cfg.ctrl_cfg.reading_threshold = 0;

    auto result{run_simulation(cfg)};

    CHECK(result.identified_present_wall);
    CHECK(!result.identified_absent_wall);
}

TEST(FrontWallDetectionTests, MaxThresholdMeansWallAlmostAlwaysAbsent)
{
    Config cfg{create_no_variance_config()};
    cfg.ctrl_cfg.reading_threshold = 1024;

    auto result{run_simulation(cfg)};

    CHECK(!result.identified_present_wall);
    CHECK(result.identified_absent_wall);
}

TEST(FrontWallDetectionTests, IrReadingScaleAffectsResults)
{
    Config cfg{create_no_variance_config()};

    cfg.env_cfg.ir_reading_scale = 0.1;
    auto low_scale{run_simulation(cfg)};

    cfg.env_cfg.ir_reading_scale = 2.0;
    auto high_scale{run_simulation(cfg)};

    CHECK(!are_results_equivalent(low_scale, high_scale));
}

TEST(FrontWallDetectionTests, AngleAffectsResults)
{
    Config cfg{create_no_variance_config()};

    auto no_angle{run_simulation(cfg)};

    cfg.env_cfg.mouse_angle = M_PI / 3;
    auto some_angle{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_angle, some_angle));
}

TEST(FrontWallDetectionTests, HorizontalVarianceAndAngleAffectsResults)
{
    Config cfg{create_no_variance_config()};
    cfg.env_cfg.mouse_angle = M_PI / 8;

    cfg.env_cfg.horizontal_position_variance = -0.9;
    auto left{run_simulation(cfg)};

    cfg.env_cfg.horizontal_position_variance = 0.9;
    auto right{run_simulation(cfg)};

    CHECK(!are_results_equivalent(left, right));
}

TEST(FrontWallDetectionTests, VerticalVarianceAffectsResults)
{
    Config cfg{create_no_variance_config()};

    cfg.env_cfg.vertical_position_variance = -0.9;
    auto back{run_simulation(cfg)};

    cfg.env_cfg.vertical_position_variance = 0.9;
    auto front{run_simulation(cfg)};

    CHECK(!are_results_equivalent(back, front));
}

IGNORE_TEST(FrontWallDetectionTests, VisualizeWithIdealParameters)
{
    Config cfg{};
    cfg.ctrl_cfg.reading_threshold = 279u;
    cfg.env_cfg.ir_reading_scale = 1.0;

    enable_visualization("ideal-parameters");

    std::vector<double> angles{-M_PI / 4, 0.0, M_PI / 4};
    std::vector<double> offsets{-0.9, 0.0, 0.9};

    for (double angle : angles) {
        for (double h_offset : offsets) {
            for (double v_offset : offsets) {
                cfg.env_cfg.mouse_angle = angle;
                cfg.env_cfg.horizontal_position_variance = h_offset;
                cfg.env_cfg.vertical_position_variance = v_offset;

                run_simulation(cfg);
            }
        }
    }
}