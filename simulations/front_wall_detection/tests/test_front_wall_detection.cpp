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

Config create_no_variance_config(void)
{
    Config cfg{};
    cfg.ir_reading_scale = {1.0};
    cfg.mouse_angle = {0.0};
    cfg.horizontal_position_variance = {0.0};
    cfg.vertical_position_variance = {0.0};

    cfg.reading_threshold = {300u};

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
TEST(FrontWallDetectionTests, ExtremeScaleTriggersClamping)
{
    Config cfg{create_no_variance_config()};

    cfg.ir_reading_scale = 100.0; /* force overflow */

    auto result{run_simulation(cfg)};

    CHECK(result.identified_present_wall == true || result.identified_present_wall == false);
}

TEST(FrontWallDetectionTests, ZeroThresholdMeansWallAlwaysPresent)
{
    Config cfg{create_no_variance_config()};
    cfg.reading_threshold = 0;

    auto result{run_simulation(cfg)};

    CHECK(result.identified_present_wall);
    CHECK(!result.identified_absent_wall);
}

TEST(FrontWallDetectionTests, MaxThresholdMeansWallAlmostAlwaysAbsent)
{
    Config cfg{create_no_variance_config()};
    cfg.reading_threshold = 1024;

    auto result{run_simulation(cfg)};

    CHECK(!result.identified_present_wall);
    CHECK(result.identified_absent_wall);
}

TEST(FrontWallDetectionTests, IrReadingScaleAffectsResults)
{
    Config cfg{create_no_variance_config()};

    cfg.ir_reading_scale = 0.1;
    auto low_scale{run_simulation(cfg)};

    cfg.ir_reading_scale = 2.0;
    auto high_scale{run_simulation(cfg)};

    CHECK(!are_results_equivalent(low_scale, high_scale));
}

TEST(FrontWallDetectionTests, AngleAffectsResults)
{
    Config cfg{create_no_variance_config()};

    auto no_angle{run_simulation(cfg)};

    cfg.mouse_angle = M_PI / 3;
    auto some_angle{run_simulation(cfg)};

    CHECK(!are_results_equivalent(no_angle, some_angle));
}

TEST(FrontWallDetectionTests, HorizontalVarianceAndAngleAffectsResults)
{
    Config cfg{create_no_variance_config()};
    cfg.mouse_angle = M_PI / 8;

    cfg.horizontal_position_variance = -0.9;
    auto left{run_simulation(cfg)};

    cfg.horizontal_position_variance = 0.9;
    auto right{run_simulation(cfg)};

    CHECK(!are_results_equivalent(left, right));
}

TEST(FrontWallDetectionTests, VerticalVarianceAffectsResults)
{
    Config cfg{create_no_variance_config()};

    cfg.vertical_position_variance = -0.9;
    auto back{run_simulation(cfg)};

    cfg.vertical_position_variance = 0.9;
    auto front{run_simulation(cfg)};

    CHECK(!are_results_equivalent(back, front));
}
